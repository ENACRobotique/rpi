#!/usr/bin/env python3

import ecal.nanobind_core as ecal_core
from ecal.msg.proto.core import Subscriber as ProtoSubscriber
from ecal.msg.proto.core import Publisher as ProtoPublisher
from ecal.msg.common.core import ReceiveCallbackData
import numpy as np
from dataclasses import dataclass
from generated.robot_state_pb2 import Aruco, Arucos
import os 
import time


@dataclass
class ArucoInfo:
    id:int
    pos:np.ndarray
    #rot:np.ndarray
    cam:str


class ArucoState:
    def __init__(self, perish_time):
        if not ecal_core.is_initialized():
            ecal_core.initialize("arucoFinder")

        self.perish_time = perish_time
        self.aruco_sub = ProtoSubscriber(Arucos, "Arucos")
        self.aruco_sub.set_receive_callback(self.aruco_cb)

        self.cam_pose =  self.get_camera_pose()
       
        self.data = {}
        self.last_update_time = {}


    def aruco_cb(self, pub_id: ecal_core.TopicId, data: ReceiveCallbackData[Arucos]):
        self.data[data.message.cameraName] = data.message.arucos
        self.last_update_time[data.message.cameraName] = time.time()
    
    def clear_old_data(self):
        for cam in self.data:
            if time.time() - self.last_update_time[cam] > self.perish_time:
                self.data[cam] = []


    @staticmethod
    def get_camera_pose():
        poses = {}
        path_to_dir = os.path.dirname(os.path.abspath(__file__))
        path_to_calib = os.path.join(path_to_dir,"../../data/camera_calibrations")
        for file in os.listdir(path_to_calib):
            if file.endswith("_rvec.npy"):
                camera_name = file.removesuffix('_rvec.npy') 
                tvec_file = f'{camera_name}_tvec.npy'
                tvec_path = os.path.join("../../data/camera_calibrations", tvec_file)
                rvec_path = os.path.join("../../data/camera_calibrations", file)

                if os.path.exists(tvec_path):
                    poses[camera_name] = (np.load(tvec_path), np.load(rvec_path)) 
        return poses
    

    def get_aruco_robot(self):
        self.clear_old_data()
        arucos = []

        for cam, ars in self.data.items():
            P_cw, R_wc = self.cam_pose[cam] 
            for ar in ars:

                P_tc = np.array([ar.x,ar.y,ar.z])
                P_tw = R_wc @ P_tc + P_cw

                arucos.append(ArucoInfo(ar.ArucoId,P_tw,cam))
        
        return arucos




if __name__ == '__main__':
    test = ArucoState(3)
    
    while True:
        testest = test.get_aruco_robot()
        print(testest)

        time.sleep(1)