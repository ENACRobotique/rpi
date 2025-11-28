#!/usr/bin/env python3
import numpy as np
from math import degrees, atan2, radians
from scipy.stats import linregress
from numpy.linalg import svd
import sys
sys.path.append('../generated')
import lidar_data_pb2  as pbl
import robot_state_pb2 as hlm
import ecal.nanobind_core as ecal_core
from ecal.msg.proto.core import Subscriber as ProtoSubscriber
from ecal.msg.proto.core import Publisher as ProtoPublisher
from ecal.msg.common.core import ReceiveCallbackData
import json
import socket
import time


plotjuggler_udp = ("192.168.42.201", 9870)
so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

d_capteurs = 100 #distance entre nos capteurs


# Le vl pour la présentation cité de l'espace est le 1

class Radar():
    def __init__(self, nb, papa) -> None:
        self.nb = nb
        self.lidar_sub = ProtoSubscriber(pbl.Lidar, f"vl53_{nb}")
        self.distance_matrix = np.empty((8,8))
        self.lidar_sub.set_receive_callback(self.get_distance)
        self.papa = papa
        self.y_best = 0
        self.r_best = 0
        self.stderr = 0
        self.angle = 0
        self.slope_best = 0

    def get_distance(self, pub_id: ecal_core.TopicId, data: ReceiveCallbackData[pbl.Lidar], ):
        msg = data.message
        pixel_angles = np.array([(i-3.5)*radians(45)/8 for i in range(8)])
        self.distances = list(msg.distances)
        distance_matrix = np.empty((8,8))
        def idx(x, y):
            return (8*x+(7-y))
        
        for y in range(8):
            for x in range(8):
                distance_matrix[y,x] = self.distances[idx(x,y)]
        self.distance_matrix = distance_matrix

        def reg_line(dists):
            xs = dists * np.sin(pixel_angles)
            ys = dists * np.cos(pixel_angles)
            return linregress(xs, ys)
        
        lin = [(y, *reg_line(self.distance_matrix[y])) for y in range(8)]
        self.y_best, slope_best, _, self.r_best, _, self.stderr = min(lin, key=lambda l: abs(l[5]))
        self.angle = np.arctan(slope_best)
        if (self.nb == 0):
            print(f"{self.y_best} -> {degrees(self.angle):+03.0f} : {slope_best:+05.2f} : {self.r_best:.2f}  sdterr:{self.stderr}")

        self.papa.calc_orientation()


class RadarDetectOrientationPlanche():
    def __init__(self, nb1, nb2):
        self.radar1 = Radar(nb1, self)
        self.radar2 = Radar(nb2, self)

    def calc_orientation(self):
        
        dist_planche1 = (self.radar1.distance_matrix[self.radar1.y_best][3] + self.radar1.distance_matrix[self.radar1.y_best][4])/2
        dist_planche2 = (self.radar2.distance_matrix[self.radar2.y_best][3] + self.radar2.distance_matrix[self.radar2.y_best][4])/2
        

        # var_deg1 = ((self.radar1.stderr / (1 + self.radar1.slope_best**2)))**2
        # var_deg2 = ((self.radar2.stderr / (1 + self.radar2.slope_best**2)))**2

        # w1 = 1.0 / var_deg1
        # w2 = 1.0 / var_deg2
        w1 = 1 / self.radar1.stderr**2
        w2 = 1 / self.radar2.stderr**2
        #print(w1)
        alpha = (w1 * self.radar1.angle + w2 * self.radar2.angle) / (w1 + w2)

        #print(f"{degrees(alpha):.2f} -> {self.radar1.r_best:.2f} -> {self.radar2.r_best:.2f}")
        d = {"Orientation_planche" : {"angle":degrees(alpha),"r1":self.radar1.r_best,"r2":self.radar2.r_best, "err1":self.radar1.stderr, "err2":self.radar2.stderr, "dist": (dist_planche1+dist_planche2)/2, "a1":degrees(self.radar1.angle), "a2":degrees(self.radar2.angle)}}
        jj = json.dumps(d)
        so.sendto(jj.encode(), plotjuggler_udp)






        
        



            

        

if __name__ == "__main__":
    if not ecal_core.is_initialized():
        ecal_core.initialize("VL53DetectOrientation")
    vl = RadarDetectOrientationPlanche(0,1)
    while True:
        pass
