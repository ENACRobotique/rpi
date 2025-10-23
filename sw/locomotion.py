#!/bin/env python3
import sys
sys.path.append('../generated')
import time
import messages_pb2 as llpb
import robot_state_pb2 as hgpb
import common_pb2 as common_pb
import math
from math import cos, sin, atan2, sqrt, pi, radians
from copy import deepcopy

import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher
from ecal.core.subscriber import ProtoSubscriber
from common import Pos, Speed, clamp, normalize_angle
from enum import Enum
from threading import Thread


class Velocity(Enum):
    FAST = (600, 4)
    NORMAL = (300, 2)
    SLOW = (100, 1)

RATE = 10
#ROBOT_RADIUS = 130.5
ACCEL_MAX = 1000
ANG_ACCEL_MAX = 4
VMAX = 600
ANG_VMAX = 2

XY_ACCURACY = 15
THETA_ACCURACY = radians(2)

FINALIZING_TIME = 1


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

        self.speed_pub = ProtoPublisher("speed_cons", common_pb.Speed)
        self.reset_pos_pub = ProtoPublisher("reset", common_pb.Position)

        self.odom_speed_sub = ProtoSubscriber("odom_speed", common_pb.Speed)
        self.odom_speed_sub.set_callback(self.on_odom_speed)

        self.target_pos_sub = ProtoSubscriber("set_position", common_pb.Position)
        self.target_pos_sub.set_callback(self.on_target_pos)
        
        self.odom_pos_sub = ProtoSubscriber("odom_pos", common_pb.Position)
        self.odom_pos_sub.set_callback(self.on_odom_pos)
        
        self.reset_pos_sub = ProtoSubscriber("reset", common_pb.Position)
        self.reset_pos_sub.set_callback(self.on_reset_pos)
        
        self.lidar_sub = ProtoSubscriber("lidar_pos", common_pb.Position)
        self.lidar_sub.set_callback(self.on_lidar_pos)

        self.kp = 3
        self.kp_ang = 1.5

        self.speed = VMAX
        
        #self.integral = Pos(0, 0, 0)
        self.lidar_pos = Pos(0,0,0)
        self.pos = Pos(0,0,0)
        self.target_pos = Pos(0,0,0)
        self.loco_state = LocoState.IDLE
        self.last_speed = Speed(0, 0, 0)
        self.odom_speed = Speed(0, 0, 0)
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
                self.speed_pub.send(common_pb.Speed(vx=0, vy=0, vtheta=0))
                self.loco_state = LocoState.IDLE
            return
        
        d_theta = normalize_angle(self.pos.theta - self.target_pos.theta)
        distance = self.pos.distance(self.target_pos)

        carrot = deepcopy(self.target_pos)

        if self.loco_state == LocoState.RUNNING:
            if distance < XY_ACCURACY and abs(d_theta) < THETA_ACCURACY:
                self.loco_state = LocoState.FINALIZING
                self.time_threshold = time.time()
                # print("Presque arrivé!")
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
                self.speed_pub.send(common_pb.Speed(vx=0, vy=0, vtheta=0))
                # print("Arrivé!")
                return
        
        # target position dans le repère robot
        dpos_r = carrot.to_frame(self.pos)
        route = atan2(dpos_r.y, dpos_r.x)

        # XY speed
        speed_cons_norm = min(distance * self.kp, self.speed)        # limit VMAX
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
    
    def is_idle(self):
        return self.loco_state == LocoState.IDLE


    def set_speed(self, speed: Speed, timeout=2):
        self.loco_state = LocoState.SPEED
        self.speed_cons_timeout = timeout
        self.time_threshold = time.time()
        self.last_speed = speed
        self.last_speed_norm = speed.xy_norm()
    
    def go_to(self, target_pos: Pos, speed=VMAX):
        self.target_pos = target_pos
        self.loco_state = LocoState.RUNNING
        self.total_dist = self.pos.distance(self.target_pos)
        # print(f"set target to {self.target_pos}")
    
    def reset_pos(self, pos: Pos):
        self.pos = pos
        # en cas de recalage on annule tous les déplacements en cours
        self.loco_state = LocoState.IDLE
        self.speed_pub.send(common_pb.Speed(vx=0, vy=0, vtheta=0))

    def on_odom_pos(self, topic, msg, timestamp):
        self.pos = Pos(msg.x, msg.y, msg.theta)
        #self.last_odom_pos_time = time.time()
    
    def on_target_pos(self, topic, msg, timestamp):
        self.go_to(Pos.from_proto(msg))
    
    def on_reset_pos(self, topic, msg, timestamp):
        self.reset_pos(Pos.from_proto(msg))
        
    
    def run(self):
        while not self.stop_requested:
            self.update()
            time.sleep(1/RATE)
    
    def stop(self):
        self.stop_requested = True
        self.join()
    
    def hasReachedTarget(self):
        return self.loco_state == LocoState.IDLE #or self.loco_state == LocoState.FINALIZING
    
    def set_move_speed(self, speed: float):
        self.speed = min(abs(speed), VMAX)
    
    def select_velocity(self, velocity: Velocity):
        self.speed, _ = velocity.value

    def on_odom_speed(self, topic_name, msg, timestamp):
        self.odom_speed = Speed.from_proto(msg)

    def on_lidar_pos(self, topic_name, msg, timestamp):
        self.lidar_pos = Pos.from_proto(msg)
        if abs(self.last_speed.vtheta) <= 0.2 and abs(self.odom_speed.vtheta) <= 0.2 :
            self.pos = self.lidar_pos
            self.reset_pos_pub.send(msg)


if __name__ == '__main__':
    loco = Locomotion()
    loco.start()
    while True:
        time.sleep(1)
