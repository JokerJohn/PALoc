#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="$(cd "${PACKAGE_DIR}/../.." && pwd)"

BAG_PATH="${BAG_PATH:-${1:-/home/xchu/data/slamData/office.bag}}"
SAVE_DIRECTORY="${SAVE_DIRECTORY:-${2:-${WS_DIR}/results/paloc_mid360_gt}}"
SEQUENCE="${SEQUENCE:-office}"
PRIOR_MAP_DIRECTORY="${PRIOR_MAP_DIRECTORY:-/home/xchu/data/slamData/gt_voxel_0p2_ltloc/}"
PLAY_RATE="${PLAY_RATE:-2.0}"
BAG_START_DELAY="${BAG_START_DELAY:-5}"
RVIZ="${RVIZ:-false}"
FASTLIO_RVIZ="${FASTLIO_RVIZ:-false}"
POST_PLAYBACK_SETTLE_SEC="${POST_PLAYBACK_SETTLE_SEC:-45}"
SAVE_TIMEOUT_SEC="${SAVE_TIMEOUT_SEC:-120}"
ROSLAUNCH_LOG="${ROSLAUNCH_LOG:-${SAVE_DIRECTORY}/office_mid360_fs_imu.launch.log}"

source /opt/ros/noetic/setup.bash
if [[ -f "${WS_DIR}/devel/setup.bash" ]]; then
  source "${WS_DIR}/devel/setup.bash"
fi

mkdir -p "${SAVE_DIRECTORY}"

LAUNCH_PID=""
cleanup() {
  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -INT "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

echo "[paloc-run] launching PALoc office MID360(FS IMU) pipeline"
roslaunch paloc office_mid360_fs_imu.launch \
  rviz:="${RVIZ}" \
  fastlio_rviz:="${FASTLIO_RVIZ}" \
  sequence:="${SEQUENCE}" \
  bag_path:="${BAG_PATH}" \
  play_rate:="${PLAY_RATE}" \
  bag_start_delay:="${BAG_START_DELAY}" \
  save_directory:="${SAVE_DIRECTORY}/" \
  prior_map_directory:="${PRIOR_MAP_DIRECTORY}" \
  >"${ROSLAUNCH_LOG}" 2>&1 &
LAUNCH_PID=$!

echo "[paloc-run] waiting for /save_map service"
until rosservice info /save_map >/dev/null 2>&1; do
  sleep 1
done

BAG_DURATION_SEC="$(rosbag info "${BAG_PATH}" | awk -F'[()]' '/duration:/ {gsub(/s/, "", $2); print $2; exit}')"
if [[ -z "${BAG_DURATION_SEC}" ]]; then
  echo "[paloc-run] failed to parse rosbag duration from ${BAG_PATH}" >&2
  exit 1
fi

WAIT_SEC="$(python3 - <<PY
bag_dur = float("${BAG_DURATION_SEC}")
play_rate = float("${PLAY_RATE}")
start_delay = float("${BAG_START_DELAY}")
settle = float("${POST_PLAYBACK_SETTLE_SEC}")
print(max(1.0, start_delay + bag_dur / play_rate + settle))
PY
)"

echo "[paloc-run] bag_duration=${BAG_DURATION_SEC}s play_rate=${PLAY_RATE} wait=${WAIT_SEC}s"
sleep "${WAIT_SEC}"

echo "[paloc-run] calling /save_map"
rosservice call /save_map "{}" >/dev/null

SEQ_DIR="${SAVE_DIRECTORY%/}/${SEQUENCE}"
INPUT_TUM="${SEQ_DIR}/optimized_poses_tum.txt"
OUTPUT_TUM="${SEQ_DIR}/optimized_poses_fs_lidar.tum"
OUTPUT_PCD="${SEQ_DIR}/optimized_poses_fs_lidar_traj.pcd"

echo "[paloc-run] waiting for trajectory file"
for _ in $(seq 1 "${SAVE_TIMEOUT_SEC}"); do
  if [[ -f "${INPUT_TUM}" ]]; then
    break
  fi
  sleep 1
done

if [[ ! -f "${INPUT_TUM}" ]]; then
  echo "[paloc-run] missing trajectory file after save: ${INPUT_TUM}" >&2
  echo "[paloc-run] roslaunch log: ${ROSLAUNCH_LOG}" >&2
  exit 1
fi

python3 "${SCRIPT_DIR}/convert_gt_tum_to_fs_frame.py" \
  --input-tum "${INPUT_TUM}" \
  --output-tum "${OUTPUT_TUM}" \
  --output-pcd "${OUTPUT_PCD}" \
  --extrinsic-config "${WS_DIR}/src/FAST_LIO/config/fs_solidstate.yaml" \
  --extrinsic-key mapping

if [[ -f "${SEQ_DIR}/icp_tum.txt" ]]; then
  python3 "${SCRIPT_DIR}/convert_gt_tum_to_fs_frame.py" \
    --input-tum "${SEQ_DIR}/icp_tum.txt" \
    --output-tum "${SEQ_DIR}/icp_fs_lidar.tum" \
    --extrinsic-config "${WS_DIR}/src/FAST_LIO/config/fs_solidstate.yaml" \
    --extrinsic-key mapping
fi

echo "[paloc-run] finished"
echo "[paloc-run] PALoc output: ${SEQ_DIR}"
echo "[paloc-run] FS lidar GT: ${OUTPUT_TUM}"
