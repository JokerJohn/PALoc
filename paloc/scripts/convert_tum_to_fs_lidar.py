#!/usr/bin/env python3

import argparse
import math
from pathlib import Path

import numpy as np


def quat_to_rot(qx, qy, qz, qw):
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 0.0:
        return np.eye(3, dtype=np.float64)
    qx /= norm
    qy /= norm
    qz /= norm
    qw /= norm
    return np.array(
        [
            [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)],
            [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)],
            [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)],
        ],
        dtype=np.float64,
    )


def rot_to_quat(rot):
    trace = np.trace(rot)
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (rot[2, 1] - rot[1, 2]) / s
        qy = (rot[0, 2] - rot[2, 0]) / s
        qz = (rot[1, 0] - rot[0, 1]) / s
    else:
        diag = np.diag(rot)
        idx = int(np.argmax(diag))
        if idx == 0:
            s = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
            qw = (rot[2, 1] - rot[1, 2]) / s
            qx = 0.25 * s
            qy = (rot[0, 1] + rot[1, 0]) / s
            qz = (rot[0, 2] + rot[2, 0]) / s
        elif idx == 1:
            s = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
            qw = (rot[0, 2] - rot[2, 0]) / s
            qx = (rot[0, 1] + rot[1, 0]) / s
            qy = 0.25 * s
            qz = (rot[1, 2] + rot[2, 1]) / s
        else:
            s = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
            qw = (rot[1, 0] - rot[0, 1]) / s
            qx = (rot[0, 2] + rot[2, 0]) / s
            qy = (rot[1, 2] + rot[2, 1]) / s
            qz = 0.25 * s
    quat = np.array([qx, qy, qz, qw], dtype=np.float64)
    quat /= np.linalg.norm(quat)
    if quat[3] < 0.0:
        quat *= -1.0
    return quat


def make_transform(translation, rotation):
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


def parse_args():
    parser = argparse.ArgumentParser(
        description="Right-multiply a TUM trajectory by the FS IMU->FS lidar extrinsic."
    )
    parser.add_argument("--input", required=True, help="Input TUM file in the source body frame.")
    parser.add_argument("--output", required=True, help="Output TUM file in FS lidar frame.")
    parser.add_argument(
        "--extrinsic-t",
        nargs=3,
        type=float,
        default=[0.02817, -0.03434, 0.02691],
        metavar=("TX", "TY", "TZ"),
        help="Translation of target lidar in source body frame. Default matches FAST_LIO fs_solidstate.yaml.",
    )
    parser.add_argument(
        "--extrinsic-q",
        nargs=4,
        type=float,
        default=[0.0, 0.0, 0.0, 1.0],
        metavar=("QX", "QY", "QZ", "QW"),
        help="Quaternion of target lidar in source body frame.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    input_path = Path(args.input)
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    extrinsic_rot = quat_to_rot(*args.extrinsic_q)
    extrinsic_tf = make_transform(np.array(args.extrinsic_t, dtype=np.float64), extrinsic_rot)

    converted = []
    with input_path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            fields = line.split()
            if len(fields) != 8:
                raise ValueError(f"Expected 8 columns in TUM line, got {len(fields)}: {line}")
            stamp = fields[0]
            tx, ty, tz, qx, qy, qz, qw = map(float, fields[1:])
            pose_rot = quat_to_rot(qx, qy, qz, qw)
            pose_tf = make_transform(np.array([tx, ty, tz], dtype=np.float64), pose_rot)
            lidar_tf = pose_tf @ extrinsic_tf
            lidar_quat = rot_to_quat(lidar_tf[:3, :3])
            converted.append(
                f"{stamp} {lidar_tf[0,3]:.15f} {lidar_tf[1,3]:.15f} {lidar_tf[2,3]:.15f} "
                f"{lidar_quat[0]:.15f} {lidar_quat[1]:.15f} {lidar_quat[2]:.15f} {lidar_quat[3]:.15f}"
            )

    with output_path.open("w", encoding="utf-8") as handle:
        handle.write("\n".join(converted))
        if converted:
            handle.write("\n")

    print(f"[convert_tum_to_fs_lidar] wrote {len(converted)} poses to {output_path}")


if __name__ == "__main__":
    main()
