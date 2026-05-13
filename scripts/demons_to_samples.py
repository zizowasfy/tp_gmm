#!/usr/bin/env python3

import os
import numpy as np
from math import sqrt
import pickle
from pathlib import Path
from rosbags.highlevel import AnyReader
from copy import deepcopy
from scipy.spatial.transform import Rotation as R
from dtw import dtw, warp

import sys
from ament_index_python.packages import get_package_share_directory
pkg_share = get_package_share_directory('tp_gmm')
sys.path.append(os.path.join(pkg_share, 'include'))
from pClass import p
from sClass import s

class RosbagWrapper:
    def __init__(self, path):
        self.path = path
        self.reader = AnyReader([Path(self.path)])
        self.reader.open()
        
    def read_messages(self, topics):
        connections = [x for x in self.reader.connections if x.topic in topics]
        for connection, timestamp, rawdata in self.reader.messages(connections=connections):
            msg = self.reader.deserialize(rawdata, connection.msgtype)
            yield connection.topic, msg, timestamp
            
    def close(self):
        self.reader.close()

def process_demonstrations(task_name, nbFrames=2, nbStates=5, nbVar=4):
    """
    Reads demonstration rosbags, extracts poses, applies DTW to align them, 
    and returns the structured data for TP-GMM learning.
    """
    # Assuming WS_DIR/Trajectory_Data_Collection
    # Alternatively we can use pkg_share/data or an absolute path
    WS_DIR = Path("/home/zizo/the_folder")
    demons_dir = str(WS_DIR / f'Trajectory_Data_Collection/Demons/{task_name}') + '/'
    
    # Path for saving demons_info.pkl
    tasks_dir = WS_DIR / "ws_moveit/src/tp_gmm/tasks"
    os.makedirs(tasks_dir / f'{task_name}', exist_ok=True)

    demons_names = []
    demons_nums = []
    for filename in os.listdir(demons_dir):
        if filename.endswith('.bag'):
            demons_names.append(filename)
            demons_nums.append(filename[filename.find('demon'):filename.find('.bag')])

    nbdemons = len(demons_names)
    ref_demon = {'ref':'', 'ref_nbpoints': 0, 'nbDemons': nbdemons, 'demons_nums': demons_nums, 'nbFrames': nbFrames}

    raw_demon_data = {}

    for d in range(nbdemons):
        demon_bag = RosbagWrapper(os.path.join(demons_dir, demons_names[d]))
        
        posearray_topic = "/ur10_1/planned_trajectory/posearray"
        start_pose_topic = "/ur10_1/start_pose"
        goal_pose_topic = "/ur10_1/goal_pose"
        
        arr_frame1_b = None
        arr_frame1_A = None
        arr_frame2_b = None
        arr_frame2_A = None

        for topic, msg, t in demon_bag.read_messages(topics=[start_pose_topic, goal_pose_topic]):
            if topic == start_pose_topic:
                pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
                orient = R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
                arr_frame1_b = np.array([[0], [pos[0]], [pos[1]], [pos[2]]])
                arr_frame1_A = np.eye(4)
                arr_frame1_A[1:,1:] = orient.as_matrix()       
            elif topic == goal_pose_topic:
                pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
                orient = R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
                arr_frame2_b = np.array([[0], [pos[0]], [pos[1]], [pos[2]]])
                arr_frame2_A = np.eye(4)
                arr_frame2_A[1:,1:] = orient.as_matrix()

        dim = 1 + 3
        conc_arr_Data = np.zeros((dim,1))
        down_sample = 1
        
        for topic, msg, t in demon_bag.read_messages(topics=[posearray_topic]):
            for pose_count, pose in enumerate(msg.poses):
                arr_Data = np.array([[pose_count], [pose.position.x], [pose.position.y], [pose.position.z]])
                conc_arr_Data = np.concatenate((conc_arr_Data, arr_Data), axis=1)

        demon_bag.close()
        
        data_points = conc_arr_Data[:, 1:] # Drop the initial zero column
        
        # Keep track of longest demonstration to use as reference for DTW
        if data_points.shape[1] > ref_demon['ref_nbpoints']:
            ref_demon["ref"] = demons_nums[d]
            ref_demon["ref_nbpoints"] = data_points.shape[1]
            ref_demon["down_sample_factor"] = down_sample  

        raw_demon_data[demons_nums[d]] = {
            'data': data_points,
            'frame1_A': arr_frame1_A,
            'frame1_b': arr_frame1_b,
            'frame2_A': arr_frame2_A,
            'frame2_b': arr_frame2_b
        }

    # DTW Alignment
    ref_num = ref_demon['ref']
    ref_data = raw_demon_data[ref_num]['data']
    demons_nums_dtw = []
    
    # Store processed slist objects for GMM fitting
    slist = []

    for d in range(nbdemons):
        d_num = demons_nums[d]
        temp_data = raw_demon_data[d_num]['data']
        
        # Align using DTW
        DTW = dtw(temp_data[1:,:].T, ref_data[1:,:].T)
        wq = warp(DTW, index_reference=False)
        
        warped_data = temp_data[:, wq]
        warped_data[0, :] = ref_data[0, :ref_data.shape[1]]
        
        nbData = warped_data.shape[1]
        
        # Build pmat for this demonstration
        pmat = np.empty(shape=(nbFrames, nbData), dtype=object)
        
        f1_A = raw_demon_data[d_num]['frame1_A']
        f1_b = raw_demon_data[d_num]['frame1_b']
        f2_A = raw_demon_data[d_num]['frame2_A']
        f2_b = raw_demon_data[d_num]['frame2_b']
        
        # Frame 1 and 2 pre-computations (Inverse A stays the same as A is constant for the whole trajectory)
        inv_f1_A = np.linalg.inv(f1_A)
        inv_f2_A = np.linalg.inv(f2_A)
        
        for k in range(nbData):
            # pClass expects (A, b, invA, nbStates)
            pmat[0, k] = p(f1_A, f1_b, inv_f1_A, nbStates)
            pmat[1, k] = p(f2_A, f2_b, inv_f2_A, nbStates)

        slist.append(s(pmat, warped_data, nbData, nbStates))
        demons_nums_dtw.append(d_num)

    ref_demon["demons_nums"] = demons_nums_dtw
    ref_demon["nbDemons"] = len(demons_nums_dtw)
    
    with open(tasks_dir / f'{task_name}/demons_info.pkl', 'wb') as fp:
        pickle.dump(ref_demon, fp)
        
    return ref_demon, slist

if __name__ == "__main__":
    task = "pick"
    ref_demon, slist = process_demonstrations(task)
    print("Reference Demon:", ref_demon)
    print(f"Processed {len(slist)} demonstrations.")
