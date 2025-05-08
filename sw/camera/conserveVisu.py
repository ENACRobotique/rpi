#!/usr/bin/env python3
import numpy as np
import time
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
import sys
sys.path.append("../../..")
import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber
from ecal.core.publisher import ProtoPublisher
from robot_state_pb2 import Position_aruco

DIM = 500
delta = 30 # écart entre les caméras en mm
conserveradius = 73/2 # mm

class visu:
    def __init__(self):
        if not ecal_core.is_initialized():
                ecal_core.initialize(sys.argv, "arucoVisu")

        aruco_sub = ProtoSubscriber("Arucos", Position_aruco)
        aruco_sub.set_callback(self.getAruco)
        self.MabelArucos = []
        self.DipperArucos = []

    def getAruco(self, topic_name, msg, time):
        camName = msg.cameraName
        ids = np.array(msg.index)
        xs = np.array(msg.x)
        ys = np.array(msg.y)
        zs = np.array(msg.z)
        Arucos = np.array(msg.ArucoId)
        
        if camName == "dipper":
            self.DipperArucos = [ids, xs, ys, zs, Arucos]
        else:
            self.MabelArucos = [ids, xs, ys, zs, Arucos]


    def init(self):
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(5, 5))

    def visualize(self):
        # Fenêtre persistante de matplotlib
        self.ax.clear()

        # Actualiser la vue matplotlib
        self.ax.set_title("Vue de dessus")
        self.ax.set_xlabel("y (mm)")
        self.ax.set_ylabel("x (mm)")
        self.ax.set_xlim(-DIM/2, DIM/2)
        self.ax.set_ylim(0, DIM)
        self.ax.grid(True)
        self.ax.plot(-delta, 0, 'o', label='Dipper', color = "blue")
        self.ax.plot(delta, 0, 'o', label='Mabel', color = "pink")

        # Aruco visibles
        if self.DipperArucos != []:
            xs = self.DipperArucos[1]
            ys = self.DipperArucos[2]
            for i in range(len(xs)):
                self.ax.add_patch(plt.Circle((ys[i]-delta,xs[i]-delta), conserveradius, color='blue', alpha=0.5))
        
        if self.MabelArucos != []:
            xs = self.MabelArucos[1]
            ys = self.MabelArucos[2]
            for i in range(len(xs)):
                self.ax.add_patch(plt.Circle((ys[i]-delta,xs[i]-delta), conserveradius, color='blue', alpha=0.5))
                 
        self.ax.legend()
        plt.pause(0.001)

if __name__ == "__main__":
    v = visu()
    v.init()
    while ecal_core.ok():
        v.visualize()