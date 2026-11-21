"""
Helpers shared by the Lab 6 scripts. You shouldn't need to change anything here.
"""
import csv
import json
import os
from dataclasses import dataclass

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

HERE = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(HERE, 'output')
CALIBRATION_PATH = os.path.join(HERE, 'calibration', 'calibration.json')

# Where the camera is on the robot (base_link -> camera optical frame, which is
# z forward, x right, y down). The translation is the same as in Lab 4's
# tag_launch.py. We measured the cameras on the lab robots and they're tilted
# up by about 8 degrees, so that's included here (tag_launch.py assumes level).
MOUNT_TRANSLATION = np.array([0.115, 0.0, 0.0])
MOUNT_PITCH_DEG = 8.12  # positive = tilted up
MOUNT_ROLL_DEG = -0.17


def _mount_rotation():
    level = Rotation.from_euler('xyz', [-np.pi / 2, 0.0, -np.pi / 2]).as_matrix()
    tilt = Rotation.from_euler('xyz', [MOUNT_PITCH_DEG, 0.0, MOUNT_ROLL_DEG], degrees=True).as_matrix()
    return level @ tilt


T_BASE_CAM = np.eye(4)
T_BASE_CAM[:3, :3] = _mount_rotation()
T_BASE_CAM[:3, 3] = MOUNT_TRANSLATION


@dataclass
class View:
    name: str
    image: np.ndarray       # BGR
    T_map_base: np.ndarray  # 4x4 pose of base_link in the map


def pose_from_row(row):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([float(row[k]) for k in ('qx', 'qy', 'qz', 'qw')]).as_matrix()
    T[:3, 3] = [float(row[k]) for k in ('x', 'y', 'z')]
    return T


def load_run(run_dir):
    # all the images in a capture_pose folder along with their poses
    path = os.path.join(run_dir, 'poses.csv')
    if not os.path.exists(path):
        raise SystemExit(f'no poses.csv in {run_dir}. give it the folder capture_pose made, '
                         f'like ../captures/run_20261015_141502')
    views = []
    with open(path) as f:
        for row in csv.DictReader(f):
            image = cv2.imread(os.path.join(run_dir, row['file']))
            if image is None:
                raise SystemExit(f'couldn\'t read {row["file"]} in {run_dir}')
            views.append(View(row['file'], image, pose_from_row(row)))
    print(f'loaded {len(views)} images from {run_dir}')
    return views


def load_calibration(path=CALIBRATION_PATH):
    if not os.path.exists(path):
        raise SystemExit(f'{path} doesn\'t exist yet, run calibrate.py first')
    with open(path) as f:
        c = json.load(f)
    return np.array(c['K'], float), np.array(c['dist'], float)


def run_output_dir(run_dir):
    out = os.path.join(OUTPUT_DIR, os.path.basename(os.path.normpath(run_dir)))
    os.makedirs(out, exist_ok=True)
    return out


def rotation_angle_deg(R):
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1.0, 1.0))))


def write_ply(path, points, colors):
    # ascii ply, opens in MeshLab / CloudCompare
    with open(path, 'w') as f:
        f.write('ply\nformat ascii 1.0\n')
        f.write(f'element vertex {len(points)}\n')
        f.write('property float x\nproperty float y\nproperty float z\n')
        f.write('property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n')
        for (x, y, z), (r, g, b) in zip(points, np.asarray(colors).astype(int)):
            f.write(f'{x:.5f} {y:.5f} {z:.5f} {r} {g} {b}\n')
