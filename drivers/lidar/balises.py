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

TOLERANCE_PHI_MOINS_1 = 50
TOLERANCE_T_MOINS_1 = 50
ERR_TOLERANCE_BEACON_DETECT = 1

old_pos=[
         0, #x
         0, #y
         0 #phi heading
         ]#position à l'instant n-1 du robot dans le repère de la table

coords_init=[(0,0),(0,1),(1,0)]
beacons_polar_angles=[0,2*np.pi/6,np.pi]





def dist_2_pts(pt1, pt2):
    return math.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)

def trilateration(beacons_abs_position,distances):
        (x1,y1),(x2,y2),(x3,y3)=beacons_abs_position
        d1,d2,d3=distances
        A=np.array([
            [2*(x2-x1),2*(y2-y1)],
            [2*(x3-x1),2*(y3-y1)]
        ])
        B=np.array([
            [d1**2-d2**2-x1**2-y1**2+x2**2+y2**2]
            [d1**2-d3**2-x1**2-y1**2+x3**2+y3**2]
       ])
        position = np.linalg.solve(A,B)
        return position[0][0], position[1][0]

def get_heading(beacons_polar_angles,angles) : #angles c les angles mesurées par le lidar
        phi=sum(beacons_polar_angles[i]-angles[i] for i in range(3))/3    #On prend la moyenne pour qu'il y ait moins d'erreurs
        phi=np.arctan2(np.sin(phi),np.cos(phi))
        return phi

class Balises:
    def __init__(self):
        self.coords_c_rr=coords_init #coordonnées cartésiennes des balises réferentiel robot
        self.odom_pos=(0,0)
        ecal_core.initialize(sys.argv, "lid_amalgameur")
        self.sub_lidar = ProtoSubscriber("amalgames", lidar_pb.Amalgames)
        self.sub_lidar.set_callback(self.beacon_detection)
        self.old_pos_robot=old_pos

       # self.pub_coord_beacons = ProtoPublisher("coord beacons", lidar_pb.Coord_beacons)

    def beacon_detection(self, topic_name, msg, time):
        x = np.array(msg.x)
        y = np.array(msg.y)
        points = np.column_stack((x,y))

        d1_b = dist_2_pts(self.coords_c_rr[0], self.coords_c_rr[1])
        d2_b = dist_2_pts(self.coords_c_rr[0], self.coords_c_rr[2])
        d3_b = dist_2_pts(self.coords_c_rr[1], self.coords_c_rr[2])
        
        
        distance_balise = sorted([d1_b, d2_b, d3_b])

        for triplets in combinations(points, 3):


            distances_triplet = sorted([d1_b, d2_b, d3_b])
                    
            last_x,last_y,last_phi=self.old_pos_robot
            if dist_2_pts(trilateration(coords_init,self.coords_c_rr),(self.odom_pos))<= TOLERANCE_T_MOINS_1 and get_heading(beacons_polar_angles,angles=(0,0,0))-last_phi <TOLERANCE_PHI_MOINS_1:       #il faut récupérer les angles depuis les données lida     
                if all(abs(distance_balise[i] - distances_triplet[i]) <= ERR_TOLERANCE_BEACON_DETECT for i in range(3)):
                    return self.coords_c_rr
                


         

            

       # msg = lidar_pb.Coord_beacons(b1 = self.coords_c_rr[0], b2 = self.coords_c_rr[1], b3 = self.coords_c_rr[2])
       # self.pub_coord_beacons.send(msg)


    


