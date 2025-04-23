#!/bin/env python3
import sys
sys.path.append('../generated')
import time
import messages_pb2 as llpb
import robot_state_pb2 as hgpb
import math
from math import cos, sin, atan2, sqrt, pi, radians
from copy import deepcopy

import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher
from ecal.core.subscriber import ProtoSubscriber
from common import Pos, Speed, clamp, normalize_angle
from enum import Enum
from threading import Thread

RATE = 10
#ROBOT_RADIUS = 130.5
ACCEL_MAX = 500
ANG_ACCEL_MAX = 3
VMAX = 300
ANG_VMAX = 2

XY_ACCURACY = 20
THETA_ACCURACY = radians(2)

FINALIZING_TIME = 2


class LocoState(Enum):
    IDLE = 0
    RUNNING = 1
    FINALIZING = 2  # convergence de précision.
    SPEED = 3       # direct speed mode


class Locomotion(Thread):
    def __init__(self):
        Thread.__init__(self)
        if not ecal_core.is_initialized():
            ecal_core.initialize(sys.argv, "locomotion")
        self.speed_pub = ProtoPublisher("speed_cons", hgpb.Speed)
        self.target_pos_sub = ProtoSubscriber("set_position", hgpb.Position)
        self.odom_pos_sub = ProtoSubscriber("odom_pos", hgpb.Position)
        self.reset_pos_sub = ProtoSubscriber("reset", hgpb.Position)
        self.odom_pos_sub.set_callback(self.on_odom_pos)
        self.target_pos_sub.set_callback(self.on_target_pos)
        self.reset_pos_sub.set_callback(self.on_reset_pos)

        self.kp = 4
        self.kp_ang = 2
        
        #self.integral = Pos(0, 0, 0)

        self.pos = Pos(0,0,0)
        self.target_pos = Pos(0,0,0)
        self.loco_state = LocoState.IDLE
        self.last_speed = Speed(0, 0, 0)
        self.last_speed_norm = 0
        self.total_dist = 0
        self.time_threshold = 0

        self.speed_cons_timeout = 0
        #self.last_odom_pos_time = None
        #while self.last_odom_pos_time is None:
        #    time.sleep(0.1)
        self.stop_requested = False
    
    def update(self):
        if self.loco_state == LocoState.IDLE:
            return
        
        if self.loco_state == LocoState.SPEED:
            # gestion accélération????
            if time.time() - self.time_threshold < self.speed_cons_timeout:
                self.speed_pub.send(self.last_speed.to_proto())
            else:
                self.speed_pub.send(hgpb.Speed(vx=0, vy=0, vtheta=0))
                self.loco_state = LocoState.IDLE
            return
        
        d_theta = normalize_angle(self.pos.theta - self.target_pos.theta)
        distance = self.pos.distance(self.target_pos)

        carrot = deepcopy(self.target_pos)

        if self.loco_state == LocoState.RUNNING:
            if distance < XY_ACCURACY and abs(d_theta) < THETA_ACCURACY:
                self.loco_state = LocoState.FINALIZING
                self.time_threshold = time.time()
                print("Presque arrivé!")
            elif distance > 200:
                    # progression douce du theta sur toute la trajectoire.
                    ratio = 1 - distance / self.total_dist
                    theta_obj = self.pos.theta + normalize_angle(self.target_pos.theta - self.pos.theta)*ratio
                    carrot.theta = theta_obj
        
        if self.loco_state == LocoState.FINALIZING:
            # à peu près arrivé, mais on maintien l'asservissement
            # pendant quelques secondes le temps de bien converger
            if time.time() - self.time_threshold > FINALIZING_TIME:
                self.loco_state = LocoState.IDLE
                self.speed_pub.send(hgpb.Speed(vx=0, vy=0, vtheta=0))
                print("Arrivé!")
                return
        
        # target position dans le repère robot
        dpos_r = carrot.to_frame(self.pos)
        route = atan2(dpos_r.y, dpos_r.x)

        # XY speed
        speed_cons_norm = min(distance * self.kp, VMAX)        # limit VMAX
        min_speed = self.last_speed_norm - ACCEL_MAX/RATE
        max_speed = self.last_speed_norm + ACCEL_MAX/RATE
        speed_cons_norm = clamp(min_speed, speed_cons_norm, max_speed)    # limit accel

        # angular speed
        vang = clamp(-ANG_VMAX, dpos_r.theta * self.kp_ang, ANG_VMAX)   # limit ANG_VMAX
        vang = clamp(self.last_speed.vtheta - ANG_ACCEL_MAX/RATE, vang, self.last_speed.vtheta + ANG_ACCEL_MAX/RATE)    # lmit ang accel

        speed_cons = Speed(speed_cons_norm*cos(route), speed_cons_norm*sin(route), vang)
        self.speed_pub.send(speed_cons.to_proto())
        # print(f"speed cons: {speed_cons}")
        self.last_speed = speed_cons
        self.last_speed_norm = speed_cons_norm


    def set_speed(self, speed: Speed, timeout=2):
        self.loco_state = LocoState.SPEED
        self.speed_cons_timeout = timeout
        self.time_threshold = time.time()
        self.last_speed = speed
        self.last_speed_norm = speed.xy_norm()
    
    def set_target_pos(self, target_pos: Pos):
        self.target_pos = target_pos
        self.loco_state = LocoState.RUNNING
        self.total_dist = self.pos.distance(self.target_pos)
        print(f"set target to {self.target_pos}")

    def on_odom_pos(self, topic, msg, timestamp):
        self.pos = Pos(msg.x, msg.y, msg.theta)
        #self.last_odom_pos_time = time.time()
    
    def on_target_pos(self, topic, msg, timestamp):
        self.set_target_pos(Pos.from_proto(msg))
    
    def on_reset_pos(self, topic, msg, timestamp):
        # en cas de recalage on annule tous les déplacements en cours
        self.loco_state = LocoState.IDLE
        self.speed_pub.send(hgpb.Speed(vx=0, vy=0, vtheta=0))
        
    
    def run(self):
        while not self.stop_requested:
            self.update()
            time.sleep(1/RATE)
    
    def stop(self):
        self.stop_requested = True
        self.join()

if __name__ == '__main__':
    loco = Locomotion()
    loco.start()
    while True:
        time.sleep(1)
