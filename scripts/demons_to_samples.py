#!/usr/bin/env python3

"""
Demonstrations Preprocessing and Sample Preparation for TP-GMM (ROS 2)

Reads demonstration rosbags (both ROS 2 bag directories and legacy .bag files),
extracts start/goal task parameters and Cartesian EE trajectories, applies
Dynamic Time Warping (DTW) to align trajectories, and packages them into
standard sample structures (sClass and pClass) for TP-GMM learning.

Usage:
  - Imported as a module: process_demonstrations(task_name, nbFrames, nbStates, nbVar)
  - CLI standalone: python3 demons_to_samples.py --task franka_pick_cube --train
"""

import os
import sys
import re
import math
import pickle
import argparse
import numpy as np
from pathlib import Path
from copy import deepcopy
from scipy.spatial.transform import Rotation as R
from dtw import dtw, warp

from rosbags.highlevel import AnyReader

from ament_index_python.packages import get_package_share_directory
try:
    pkg_share = get_package_share_directory('tp_gmm')
    sys.path.append(os.path.join(pkg_share, 'include'))
    sys.path.append(os.path.join(pkg_share, 'scripts'))
except Exception:
    pass

from dir_paths import get_paths, get_demonstrations_dir, get_task_dir, ensure_task_dir
paths = get_paths()
demons_root = Path(paths['demons_dir'])
tasks_root = Path(paths['tasks_dir'])

from pClass import p
from sClass import s


class RosbagWrapper:
    def __init__(self, path: Path | str):
        self.path = Path(path)
        self.reader = AnyReader([self.path])
        self.reader.open()

    @property
    def connections(self):
        return self.reader.connections

    def find_topics(self) -> tuple[str | None, str | None, str | None]:
        """
        Dynamically discovers start_pose, goal_pose, and posearray topics in the bag.
        Returns: (start_pose_topic, goal_pose_topic, posearray_topic)
        """
        start_topic = None
        goal_topic = None
        posearray_topic = None

        for conn in self.reader.connections:
            topic = conn.topic
            msgtype = conn.msgtype

            if 'start_pose' in topic and ('PoseStamped' in msgtype or 'Pose' in msgtype):
                start_topic = topic
            elif 'goal_pose' in topic and ('PoseStamped' in msgtype or 'Pose' in msgtype):
                goal_topic = topic
            elif ('posearray' in topic or 'planned_trajectory' in topic or 'trajectory' in topic) and 'PoseArray' in msgtype:
                posearray_topic = topic

        return start_topic, goal_topic, posearray_topic

    def read_messages(self, topics):
        connections = [x for x in self.reader.connections if x.topic in topics]
        for connection, timestamp, rawdata in self.reader.messages(connections=connections):
            msg = self.reader.deserialize(rawdata, connection.msgtype)
            yield connection.topic, msg, timestamp

    def close(self):
        self.reader.close()


def discover_demonstrations(demons_dir: Path) -> list[Path]:
    """
    Discovers all demonstration files or directories in demons_dir,
    and returns them naturally sorted (demon_1, demon_2, ..., demon_10).
    """
    if not demons_dir.exists():
        raise FileNotFoundError(f"Demonstrations directory does not exist: {demons_dir}")

    candidates = []
    for item in demons_dir.iterdir():
        if item.is_dir():
            # ROS 2 bag folder
            if (item / "metadata.yaml").exists() or list(item.glob("*.db3")) or list(item.glob("*.mcap")):
                candidates.append(item)
            elif item.name.startswith("demon_"):
                candidates.append(item)
        elif item.is_file():
            if item.suffix in ['.bag', '.db3', '.mcap']:
                candidates.append(item)

    def extract_index(path: Path) -> int:
        match = re.search(r'demon_(\d+)', path.name)
        return int(match.group(1)) if match else 999999

    candidates.sort(key=extract_index)
    return candidates


def process_demonstrations(task_name: str, nbFrames: int = 2, nbStates: int = 5, nbVar: int = 4):
    """
    Reads demonstration rosbags, extracts poses and Cartesian trajectories,
    applies DTW to align them, and returns structured data for TP-GMM learning.

    Args:
        task_name: Name of the task (e.g., 'franka_pick_cube', 'pick', 'place')
        nbFrames: Number of task reference frames (default: 2 -> start and goal)
        nbStates: Number of Gaussian states/components (default: 5)
        nbVar: State dimensionality (default: 4 -> [time_step, x, y, z])

    Returns:
        ref_demon (dict): Metadata describing the reference demonstration and demon list.
        slist (list[s]): List of sClass instances containing warped data and frame matrices (pmat).
    """
    demons_dir = demons_root / task_name
    task_dir = ensure_task_dir(task_name)

    demon_paths = discover_demonstrations(demons_dir)
    nbdemons = len(demon_paths)

    if nbdemons == 0:
        raise RuntimeError(f"No demonstrations found in {demons_dir}")

    demons_names = [d_path.name for d_path in demon_paths]
    demons_nums = []
    for d_path in demon_paths:
        m = re.search(r'demon_\d+', d_path.name)
        demons_nums.append(m.group(0) if m else d_path.stem)

    print(f"[{task_name}] Found {nbdemons} demonstrations in {demons_dir}")

    ref_demon = {
        'ref': '',
        'ref_nbpoints': 0,
        'nbDemons': nbdemons,
        'demons_nums': demons_nums,
        'nbFrames': nbFrames,
        'down_sample_factor': 1
    }

    raw_demon_data = {}

    for d, d_path in enumerate(demon_paths):
        d_num = demons_nums[d]
        demon_bag = RosbagWrapper(d_path)

        start_topic, goal_topic, posearray_topic = demon_bag.find_topics()

        if not start_topic or not goal_topic:
            # Fallbacks
            start_topic = start_topic or "/start_pose"
            goal_topic = goal_topic or "/goal_pose"

        if not posearray_topic:
            posearray_topic = "/planned_trajectory/posearray"

        arr_frame1_b = None
        arr_frame1_A = None
        arr_frame2_b = None
        arr_frame2_A = None

        # 1. Read Start and Goal Poses (Task Parameter Frames)
        for topic, msg, t in demon_bag.read_messages(topics=[start_topic, goal_topic]):
            if topic == start_topic:
                pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
                orient = R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
                arr_frame1_b = np.array([[0], [pos[0]], [pos[1]], [pos[2]]])
                arr_frame1_A = np.eye(4)
                arr_frame1_A[1:, 1:] = orient.as_matrix()
            elif topic == goal_topic:
                pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
                orient = R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
                arr_frame2_b = np.array([[0], [pos[0]], [pos[1]], [pos[2]]])
                arr_frame2_A = np.eye(4)
                arr_frame2_A[1:, 1:] = orient.as_matrix()

        # 2. Read Trajectory Poses (take latest non-empty message to avoid concatenating duplicates)
        posearray_msgs = [msg for _, msg, _ in demon_bag.read_messages(topics=[posearray_topic])]
        demon_bag.close()

        if not posearray_msgs or len(posearray_msgs[-1].poses) == 0:
            print(f"WARNING: Demonstration '{d_path.name}' has 0 trajectory points on topic '{posearray_topic}'.")
            continue

        target_msg = posearray_msgs[-1]
        dim = 1 + 3  # [index, x, y, z]
        num_poses = len(target_msg.poses)
        data_points = np.zeros((dim, num_poses))
        for pose_count, pose in enumerate(target_msg.poses):
            data_points[:, pose_count] = [pose_count, pose.position.x, pose.position.y, pose.position.z]

        # Keep track of longest demonstration to use as DTW reference
        if data_points.shape[1] > ref_demon['ref_nbpoints']:
            ref_demon["ref"] = d_num
            ref_demon["ref_nbpoints"] = data_points.shape[1]

        raw_demon_data[d_num] = {
            'data': data_points,
            'frame1_A': arr_frame1_A if arr_frame1_A is not None else np.eye(4),
            'frame1_b': arr_frame1_b if arr_frame1_b is not None else np.zeros((4, 1)),
            'frame2_A': arr_frame2_A if arr_frame2_A is not None else np.eye(4),
            'frame2_b': arr_frame2_b if arr_frame2_b is not None else np.zeros((4, 1)),
        }

    # 3. DTW Alignment across all demonstrations
    ref_num = ref_demon['ref']
    if not ref_num or ref_num not in raw_demon_data:
        raise RuntimeError("Failed to select a valid reference demonstration for DTW alignment.")

    ref_data = raw_demon_data[ref_num]['data']
    demons_nums_dtw = []
    slist = []

    for d_num in raw_demon_data.keys():
        temp_data = raw_demon_data[d_num]['data']

        # Align using Dynamic Time Warping (DTW)
        DTW = dtw(temp_data[1:, :].T, ref_data[1:, :].T)
        wq = warp(DTW, index_reference=False)

        warped_data = temp_data[:, wq]
        warped_data[0, :] = ref_data[0, :warped_data.shape[1]]

        nbData = warped_data.shape[1]

        # Build pmat for this demonstration
        pmat = np.empty(shape=(nbFrames, nbData), dtype=object)

        f1_A = raw_demon_data[d_num]['frame1_A']
        f1_b = raw_demon_data[d_num]['frame1_b']
        f2_A = raw_demon_data[d_num]['frame2_A']
        f2_b = raw_demon_data[d_num]['frame2_b']

        inv_f1_A = np.linalg.inv(f1_A)
        inv_f2_A = np.linalg.inv(f2_A)

        for k in range(nbData):
            pmat[0, k] = p(f1_A, f1_b, inv_f1_A, nbStates)
            pmat[1, k] = p(f2_A, f2_b, inv_f2_A, nbStates)

        slist.append(s(pmat, warped_data, nbData, nbStates))
        demons_nums_dtw.append(d_num)

    ref_demon["demons_nums"] = demons_nums_dtw
    ref_demon["nbDemons"] = len(demons_nums_dtw)

    demons_info_file = task_dir / "demons_info.pkl"
    with open(demons_info_file, 'wb') as fp:
        pickle.dump(ref_demon, fp)

    print(f"[{task_name}] Reference Demonstration: '{ref_demon['ref']}' ({ref_demon['ref_nbpoints']} points).")
    print(f"[{task_name}] demons_info saved to: {demons_info_file}")

    return ref_demon, slist


def train_tpgmm(task_name: str, nbStates: int = 5, nbFrames: int = 2, nbVar: int = 4):
    """Processes demonstrations and fits a TPGMM_GMR model, saving TPGMM_model.pkl."""
    from TPGMM_GMR import TPGMM_GMR

    task_dir = ensure_task_dir(task_name)
    ref_demon, slist = process_demonstrations(task_name, nbFrames=nbFrames, nbStates=nbStates, nbVar=nbVar)

    print(f"\n--- Training TP-GMM Model for task '{task_name}' ({nbStates} States, {nbFrames} Frames) ---")
    tpgmm = TPGMM_GMR(nbStates, nbFrames, nbVar)
    tpgmm.fit(slist)

    model_file = task_dir / "TPGMM_model.pkl"
    with open(model_file, 'wb') as fp:
        pickle.dump(tpgmm, fp)

    print(f"[{task_name}] TP-GMM model trained and saved to: {model_file}\n")
    return tpgmm


def main():
    parser = argparse.ArgumentParser(description="Demonstration to Samples Processor & TP-GMM Trainer")
    parser.add_argument('--task', '-t', type=str, default='franka_pick_cube', help='Task name (default: franka_pick_cube)')
    parser.add_argument('--train', action='store_true', help='Fit and save TP-GMM model after processing demonstrations')
    parser.add_argument('--states', '-k', type=int, default=5, help='Number of Gaussian states (default: 5)')
    parser.add_argument('--frames', '-f', type=int, default=2, help='Number of task frames (default: 2)')

    args = parser.parse_args()

    # If task doesn't exist yet, fall back to pick if available
    task_demon_dir = demons_root / args.task
    if not task_demon_dir.exists() and (demons_root / 'pick').exists():
        print(f"Note: '{task_demon_dir}' does not exist yet. Defaulting to 'pick' for demonstration.")
        task = 'pick'
    else:
        task = args.task

    if args.train:
        train_tpgmm(task, nbStates=args.states, nbFrames=args.frames)
    else:
        ref_demon, slist = process_demonstrations(task, nbFrames=args.frames, nbStates=args.states)
        print(f"Processed {len(slist)} demonstrations successfully.")


if __name__ == "__main__":
    main()
