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
import robot_state_pb2 as hgpb


# il faut Créer lidar_pb.Coord_beacons 


TOLERANCE_T_MOINS_1 = 50
ERR_TOLERANCE_BEACON_DETECT = 1

def dist_2_pts(pt1, pt2):
    return math.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)

def trilateration(triplet_pts,d1,d2,d3):
        p1=triplet_pts[0]
        p2=triplet_pts[1]
        p3=triplet_pts[2]

        
        return (1,1)  
coords_init=[(0,0),(0,1),(1,0)]
class Balises:
    def __init__(self):
        self.coords_c_rr=coords_init #coordonnées cartésiennes des balises réferentiel robot
        self.odom_pos=(0,0)
        ecal_core.initialize(sys.argv, "lid_amalgameur")
        self.sub_lidar = ProtoSubscriber("amalgames", lidar_pb.Amalgames)
        ecal_core.initialize(sys.argv, "Bridge low level")
        self.odom_pos_pub = ProtoSubscriber("odom_pos", hgpb.Position)
        self.odom_pos_pub.set_callback(self.odompos)
        self.sub_lidar.set_callback(self.beacon_detection)
       # self.pub_coord_beacons = ProtoPublisher("coord beacons", lidar_pb.Coord_beacons)
    def odompos(self,topic_name,msg,time):
        self.odom_pos=(msg.x,msg.y)

    def beacon_detection(self, topic_name, msg, time):
        x = np.array(msg.x)
        y = np.array(msg.y)
        points = np.column_stack((x,y))

        d1_b = dist_2_pts(self.coords_c_rr[0], self.coords_c_rr[1])
        d2_b = dist_2_pts(self.coords_c_rr[0], self.coords_c_rr[2])
        d3_b = dist_2_pts(self.coords_c_rr[1], self.coords_c_rr[2])
        
        
        distance_balise = sorted([d1_b, d2_b, d3_b])

        for triplets in combinations(points, 3):


            distances_triplet = sorted([d1, d2, d3])
                    
                
            if dist_2_pts(trilateration(triplets),self.odom_pos)<= TOLERANCE_T_MOINS_1:
                d1 = dist_2_pts(triplets[0], triplets[1])  
                d2 = dist_2_pts(triplets[0], triplets[2])
                d3 = dist_2_pts(triplets[1], triplets[2])

            
                if all(abs(distance_balise[i] - distances_triplet[i]) <= ERR_TOLERANCE_BEACON_DETECT for i in range(3)):
                    return self.coords_c_rr
                


         

            

       # msg = lidar_pb.Coord_beacons(b1 = self.coords_c_rr[0], b2 = self.coords_c_rr[1], b3 = self.coords_c_rr[2])
       # self.pub_coord_beacons.send(msg)


    


