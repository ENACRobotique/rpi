from itertools import combinations
import ecal.core.core as ecal_core
import math
from ecal.core.subscriber import ProtoSubscriber
from ecal.core.publisher import ProtoPublisher
import numpy as np
import time, os, sys
from numpy import typing as npt
import generated.robot_state_pb2 as robot_pb
import generated.lidar_data_pb2 as lidar_pb

# il faut Créer lidar_pb.Coord_beacons 


TOLERANCE_T_MOINS_1 = 50
ERR_TOLERANCE_BEACON_DETECT = 1

def dist_2_pts(pt1, pt2):
    return math.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)


coords_init=[(0,0),(0,1),(1,0)]
class Balises:
    def __init__(self):
        self.coords_c_rr=coords_init #coordonnées cartésiennes des balises réferentiel robot
        ecal_core.initialize(sys.argv, "lid_amalgameur")

        self.sub_lidar = ProtoSubscriber("amalgames", lidar_pb.Amalgames)
        self.sub_lidar.set_callback(self.beacon_detection)
        self.pub_coord_beacons = ProtoPublisher("coord beacons", lidar_pb.Coord_beacons)

    def beacon_detection(self, topic_name, msg, time):
        x = np.array(msg.x)
        y = np.array(msg.y)
        points = np.column_stack(x, y)

        d1_b = dist_2_pts(self.coords_c_rr[0], self.coords_c_rr[1])
        d2_b = dist_2_pts(self.coords_c_rr[0], self.coords_c_rr[2])
        d3_b = dist_2_pts(self.coords_c_rr[1], self.coords_c_rr[2])
        
        
        distance_balise = sorted([d1_b, d2_b, d3_b])

        for triplets in combinations(points, 3):
            d1 = dist_2_pts(triplets[0], triplets[1])  
            d2 = dist_2_pts(triplets[0], triplets[2])
            d3 = dist_2_pts(triplets[1], triplets[2])

            

            distances_triplet = sorted([d1, d2, d3])

            if all(abs(distance_balise[i] - distances_triplet[i]) <= ERR_TOLERANCE_BEACON_DETECT for i in range(3)):
                #comparer le nouveau triplet de coord avec l'ancien pour ne pas accepter des nouvelles balises incohérentes
                a0 = dist_2_pts(triplets[0], self.coords_c_rr[0])
                b0 = dist_2_pts(triplets[0], self.coords_c_rr[1])
                c0 = dist_2_pts(triplets[0], self.coords_c_rr[2])

                a1 = dist_2_pts(triplets[1], self.coords_c_rr[0])
                b1 = dist_2_pts(triplets[1], self.coords_c_rr[1])
                c1 = dist_2_pts(triplets[1], self.coords_c_rr[2])

                a2 = dist_2_pts(triplets[2], self.coords_c_rr[0])
                b2 = dist_2_pts(triplets[2], self.coords_c_rr[1])
                c2 = dist_2_pts(triplets[2], self.coords_c_rr[2])

                if min(a0, b0, c0) + min(a1, b1, c1) + min(a2, b2, c2) <= TOLERANCE_T_MOINS_1
                    self.coords_c_rr = triplets
                    return self.coords_c_rr
            

        msg = lidar_pb.Coord_beacons(b1 = self.coords_c_rr[0], b2 = self.coords_c_rr[1], b3 = self.coords_c_rr[2])
        self.pub_coord_beacons.send(msg)


    


