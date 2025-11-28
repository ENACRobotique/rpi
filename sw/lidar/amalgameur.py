#!/usr/bin/env python3
import ecal.nanobind_core as ecal_core
from ecal.msg.proto.core import Subscriber as ProtoSubscriber
from ecal.msg.proto.core import Publisher as ProtoPublisher
from ecal.msg.common.core import ReceiveCallbackData
import time, os, sys
sys.path.append(os.path.join(os.path.dirname(__file__), '../..')) # Avoids ModuleNotFoundError when finding generated folder
import numpy as np
from numpy import typing as npt
import generated.robot_state_pb2 as robot_pb
import generated.lidar_data_pb2 as lidar_pb

QUALITY_REJECTION_VAL = 160
MAX_DIST = 3600 # diagonal of the table in mm
AMALGAME_DIST_THRESHOLD_SQUARE = 150**2
MAX_SIZE = 500


class Amalgameur:
    def __init__(self) -> None:
        ecal_core.initialize("lidar_amalgameur")

        self.sub_lidar = ProtoSubscriber(lidar_pb.Lidar, "lidar_data")
        self.sub_lidar.set_receive_callback(self.on_lidar)
        self.pub_amalgames = ProtoPublisher(lidar_pb.Amalgames, "amalgames")

    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_value, traceback):
        self.sub_lidar.remove_receive_callback()


    def on_lidar(self, pub_id : ecal_core.TopicId, data : ReceiveCallbackData[lidar_pb.Lidar]):
        quality = np.array(data.message.quality) > QUALITY_REJECTION_VAL
        distances = np.array(data.message.distances)[quality]
        angles = np.array(data.message.angles)[quality]

        distance_filter = distances < MAX_DIST

        distances = distances[distance_filter]
        angles = angles[distance_filter]
        
        #print(len(distances), "Dist", distances)
        
        amalgames = self.find_amalgames(distances, angles)

        
        xs, ys, sizes = [], [], []
        for a in amalgames:
            x, y, size = self.get_amalgame_infos(distances[a], angles[a])
            if size < MAX_SIZE:
                xs.append(x)
                ys.append(y)
                sizes.append(size)
        msg = lidar_pb.Amalgames(x=xs, y=ys, size=sizes)
        self.pub_amalgames.send(msg)

            #print(f"{x:.0f} {y:.0f} {size:.0f}")
        #print("-------------")
    
    @staticmethod
    def get_amalgame_infos(distances, angles):
        if len(distances) == 1:
            size = 0
        else:
            p0 = distances[0], angles[0]
            p1 = distances[-1], angles[-1]
            d2 = Amalgameur.get_squared_dist_polar(p0, p1)
            size = np.sqrt(d2)
        i_center = len(distances) // 2
        d = distances[i_center]
        ar = angles[i_center]
        x = (d+50) * np.cos(ar)
        y = (d+50) * np.sin(ar)
        return x, y, size

    def find_amalgames(self, distances, angles) -> list[np.ndarray]:
        """
        return list of np.array of indices
        """
        amalgames = []
        if len(distances) < 2:
            return []
        new_amalgame = [0]

        for i in range(1, len(distances) - 1):
            last_pt = distances[i-1], angles[i-1]
            cur_pt = distances[i], angles[i]
            dist = self.get_squared_dist_polar(last_pt, cur_pt)
            if dist < AMALGAME_DIST_THRESHOLD_SQUARE:
                new_amalgame.append(i)
            else:
                amalgames.append(new_amalgame)
                new_amalgame = [i]
        amalgames.append(new_amalgame)

        # handle wrap
        if len(amalgames) > 1:
            i0 = amalgames[0][0]
            i1 = amalgames[-1][-1]
            p0 = distances[i0], angles[i0]
            p1 = distances[i1], angles[i1]
            if self.get_squared_dist_polar(p0, p1) < AMALGAME_DIST_THRESHOLD_SQUARE:
                amalgames[-1].extend(amalgames[0])
                del amalgames[0]
        
        return [np.array(a) for a in amalgames]
    
    @staticmethod
    def get_squared_dist_polar(pt1, pt2):
        """
        https://math.stackexchange.com/questions/1506706/how-to-calculate-the-distance-between-two-points-with-polar-coordinates
        """
        r1, theta1 = pt1
        r2, theta2 = pt2
        return r1**2 + r2**2 - 2 * r1 * r2 * np.cos(theta2 - theta1)


if __name__ == "__main__":
    with Amalgameur() as d :
        while ecal_core.ok():
            time.sleep(1)
