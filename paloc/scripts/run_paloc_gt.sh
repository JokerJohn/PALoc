#!/usr/bin/env bash
# =====================================================================
# run_paloc_gt.sh - Run PALoc ground truth generation pipeline
#
# Usage:
#   # Indoor (office)
#   bash run_paloc_gt.sh indoor /home/xchu/data/slamData/office.bag office
#
#   # Outdoor
#   bash run_paloc_gt.sh outdoor /home/xchu/data/slamData/dsh1-1.bag dsh1-1 \
#        /path/to/outdoor_prior_map/
#
# Environment overrides:
#   PLAY_RATE=2.0 RVIZ=true bash run_paloc_gt.sh indoor ...
# =====================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="$(cd "${PACKAGE_DIR}/../.." && pwd)"

SCENE="${1:?Usage: $0 <indoor|outdoor> <bag_path> <sequence> [prior_map_dir]}"
BAG_PATH="${2:?Usage: $0 <indoor|outdoor> <bag_path> <sequence> [prior_map_dir]}"
SEQUENCE="${3:?Usage: $0 <indoor|outdoor> <bag_path> <sequence> [prior_map_dir]}"
PRIOR_MAP_DIR="${4:-/home/xchu/data/slamData/gt_voxel_0p2_ltloc/}"

SAVE_DIRECTORY="${SAVE_DIRECTORY:-${WS_DIR}/results/paloc_mid360_gt}"
PLAY_RATE="${PLAY_RATE:-1.0}"
BAG_START_DELAY="${BAG_START_DELAY:-3}"
RVIZ="${RVIZ:-false}"
PALOC_CONFIG="${PALOC_CONFIG:-}"
POST_PLAYBACK_SETTLE_SEC="${POST_PLAYBACK_SETTLE_SEC:-45}"
SAVE_TIMEOUT_SEC="${SAVE_TIMEOUT_SEC:-120}"

SEQ_DIR="${SAVE_DIRECTORY}/${SEQUENCE}"
LOG_FILE="${SAVE_DIRECTORY}/${SCENE}_${SEQUENCE}.launch.log"
VIZ_SCRIPT="${WS_DIR}/src/mapping_task/ms_mapping/scripts/visualize_pose_graph.py"

case "${SCENE}" in
  indoor) LAUNCH_FILE="indoor_mid360.launch" ;;
  outdoor) LAUNCH_FILE="outdoor_mid360.launch" ;;
  *) echo "[paloc-run] ERROR: scene must be 'indoor' or 'outdoor', got '${SCENE}'" >&2; exit 1 ;;
esac

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

echo "[paloc-run] scene=${SCENE} sequence=${SEQUENCE} bag=${BAG_PATH}"
echo "[paloc-run] prior_map=${PRIOR_MAP_DIR}"
echo "[paloc-run] launching ${LAUNCH_FILE}"

LAUNCH_ARGS=(
  bag_path:="${BAG_PATH}"
  sequence:="${SEQUENCE}"
  play_rate:="${PLAY_RATE}"
  bag_start_delay:="${BAG_START_DELAY}"
  save_directory:="${SAVE_DIRECTORY}/"
  prior_map_directory:="${PRIOR_MAP_DIR}"
  rviz:="${RVIZ}"
)
if [[ -n "${PALOC_CONFIG}" ]]; then
  LAUNCH_ARGS+=( paloc_config:="${PALOC_CONFIG}" )
  echo "[paloc-run] paloc_config=${PALOC_CONFIG}"
fi

roslaunch paloc "${LAUNCH_FILE}" "${LAUNCH_ARGS[@]}" \
  >"${LOG_FILE}" 2>&1 &
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

TRAJ_FILE="${SEQ_DIR}/optimized_poses_tum.txt"
echo "[paloc-run] waiting for ${TRAJ_FILE}"
for _ in $(seq 1 "${SAVE_TIMEOUT_SEC}"); do
  [[ -f "${TRAJ_FILE}" ]] && break
  sleep 1
done

if [[ ! -f "${TRAJ_FILE}" ]]; then
  echo "[paloc-run] ERROR: trajectory file not found after save: ${TRAJ_FILE}" >&2
  echo "[paloc-run] check log: ${LOG_FILE}" >&2
  exit 1
fi

POSE_COUNT="$(wc -l < "${TRAJ_FILE}")"
echo "[paloc-run] trajectory saved: ${TRAJ_FILE} (${POSE_COUNT} poses)"

G2O_FILE="${SEQ_DIR}/pose_graph.g2o"
if [[ -f "${G2O_FILE}" ]] && [[ -f "${VIZ_SCRIPT}" ]]; then
  echo "[paloc-run] generating pose graph visualization"
  MPLBACKEND=Agg python3 "${VIZ_SCRIPT}" \
    --g2o "${G2O_FILE}" \
    --output "pose_graph.png" \
    2>/dev/null || echo "[paloc-run] WARNING: g2o visualization failed (non-fatal)"
  if [[ -f "${SEQ_DIR}/pose_graph.png" ]]; then
    echo "[paloc-run] g2o plot saved: ${SEQ_DIR}/pose_graph.png"
  fi
else
  echo "[paloc-run] skipping g2o visualization (file missing)"
fi

echo ""
echo "========================================"
echo "[paloc-run] DONE"
echo "  results dir : ${SEQ_DIR}"
echo "  trajectory  : ${TRAJ_FILE}"
echo "  g2o         : ${G2O_FILE}"
echo "  g2o plot    : ${SEQ_DIR}/pose_graph.png"
echo "  launch log  : ${LOG_FILE}"
echo "========================================"
