#!/usr/bin/env python3

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import yaml


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert PALoc MID360-IMU TUM poses into the FS LiDAR frame.")
    parser.add_argument("--input-tum", required=True, help="Input TUM trajectory in the MID360 IMU/body frame")
    parser.add_argument("--output-tum", required=True, help="Output TUM trajectory in the FS LiDAR frame")
    parser.add_argument("--output-pcd", help="Optional output trajectory PCD for visualization")
    parser.add_argument("--extrinsic-config", required=True, help="YAML containing T_mid360_imu_fs_lidar")
    parser.add_argument(
        "--extrinsic-key",
        default="mapping",
        help="Slash-separated key path to extrinsic_T/extrinsic_R in the YAML, default: mapping",
    )
    return parser.parse_args()


def load_extrinsic(config_path: Path, key_path: str) -> np.ndarray:
    data = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    node = data
    for key in key_path.split("/"):
        if key:
            node = node[key]
    t = np.asarray(node["extrinsic_T"], dtype=np.float64).reshape(3)
    r = np.asarray(node["extrinsic_R"], dtype=np.float64).reshape(3, 3)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = r
    T[:3, 3] = t
    return T


def quat_to_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    q = np.asarray([qx, qy, qz, qw], dtype=np.float64)
    q /= np.linalg.norm(q)
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def matrix_to_quat(R: np.ndarray) -> np.ndarray:
    trace = np.trace(R)
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s
    q = np.asarray([qx, qy, qz, qw], dtype=np.float64)
    return q / np.linalg.norm(q)


def transform_tum(input_path: Path, output_path: Path, T_mid360_imu_fs_lidar: np.ndarray) -> np.ndarray:
    points = []
    with input_path.open("r", encoding="utf-8") as src, output_path.open("w", encoding="utf-8") as dst:
        for line in src:
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            parts = stripped.split()
            if len(parts) < 8:
                raise ValueError(f"Invalid TUM line in {input_path}: {line.rstrip()}")
            stamp = parts[0]
            xyz = np.asarray(parts[1:4], dtype=np.float64)
            qx, qy, qz, qw = map(float, parts[4:8])

            T_map_mid360_imu = np.eye(4, dtype=np.float64)
            T_map_mid360_imu[:3, :3] = quat_to_matrix(qx, qy, qz, qw)
            T_map_mid360_imu[:3, 3] = xyz

            T_map_fs_lidar = T_map_mid360_imu @ T_mid360_imu_fs_lidar
            q_out = matrix_to_quat(T_map_fs_lidar[:3, :3])
            t_out = T_map_fs_lidar[:3, 3]
            points.append(t_out.copy())

            extra = " " + " ".join(parts[8:]) if len(parts) > 8 else ""
            dst.write(
                f"{stamp} {t_out[0]:.9f} {t_out[1]:.9f} {t_out[2]:.9f} "
                f"{q_out[0]:.9f} {q_out[1]:.9f} {q_out[2]:.9f} {q_out[3]:.9f}{extra}\n"
            )
    return np.asarray(points, dtype=np.float64)


def write_pcd(path: Path, points: np.ndarray) -> None:
    with path.open("w", encoding="utf-8") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {len(points)}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(points)}\n")
        f.write("DATA ascii\n")
        for x, y, z in points:
            f.write(f"{x:.9f} {y:.9f} {z:.9f}\n")


def main() -> None:
    args = parse_args()
    input_tum = Path(args.input_tum).expanduser().resolve()
    output_tum = Path(args.output_tum).expanduser().resolve()
    output_tum.parent.mkdir(parents=True, exist_ok=True)
    T_mid360_imu_fs_lidar = load_extrinsic(Path(args.extrinsic_config).expanduser().resolve(), args.extrinsic_key)
    points = transform_tum(input_tum, output_tum, T_mid360_imu_fs_lidar)
    if args.output_pcd:
        output_pcd = Path(args.output_pcd).expanduser().resolve()
        output_pcd.parent.mkdir(parents=True, exist_ok=True)
        write_pcd(output_pcd, points)
    print(f"[paloc] wrote FS-lidar TUM: {output_tum}")
    if args.output_pcd:
        print(f"[paloc] wrote FS-lidar trajectory PCD: {output_pcd}")


if __name__ == "__main__":
    main()
