#!/bin/env python3
import sys
sys.path.append('../generated')
import time
import messages_pb2 as llpb
import robot_state_pb2 as hgpb
import math
from math import cos, sin, atan2, sqrt

import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher
from ecal.core.subscriber import ProtoSubscriber
from common import Pos, Speed, clamp, normalize_angle
from enum import Enum

RATE = 10
ROBOT_RADIUS = 130.5
ACCEL_MAX = 1000
ANG_ACCEL_MAX = 4
VMAX = 200
ANG_VMAX = 4

XY_ACCURACY = 20
THETA_ACCURACY = 0.05


class LocoState(Enum):
    IDLE = 0
    RUNNING = 1


class Locomotion:
    def __init__(self):
        if not ecal_core.is_initialized():
            ecal_core.initialize(sys.argv, "locomotion")
        self.speed_pub = ProtoPublisher("speed_cons", hgpb.Speed)
        self.target_pos_sub = ProtoSubscriber("set_position", hgpb.Position)
        self.odom_pos_sub = ProtoSubscriber("odom_pos", hgpb.Position)
        self.carrot_pos_pub = ProtoPublisher("carrot_pos", hgpb.Position)
        self.last_odom_pos_time = None
        
        self.odom_pos_sub.set_callback(self.on_odom_pos)
        self.target_pos_sub.set_callback(self.on_target_pos)

        self.kp = 4
        self.kp_ang = 2
        
        self.integral = Pos(0, 0, 0)

        self.pos = Pos(0,0,0)
        self.target_pos = Pos(0,0,0)
        self.loco_state = LocoState.IDLE
        self.last_speed = Speed(0, 0, 0)
        self.last_speed_norm = 0
        self.traj_duration = 10

        while self.last_odom_pos_time is None:
            time.sleep(0.1)
    
    def update(self):
        if self.loco_state == LocoState.RUNNING:
            if self.pos.distance(self.target_pos) < XY_ACCURACY and abs(self.pos.theta - self.target_pos.theta) < THETA_ACCURACY:
                self.loco_state = LocoState.IDLE
                self.speed_pub.send(hgpb.Speed(vx=0, vy=0, vtheta=0))
                print("Arrivé!")
            else:
                dpos_r = self.target_pos.to_frame(self.pos)
                print(dpos_r)
                distance = dpos_r.norm()
                azimut = atan2(dpos_r.y, dpos_r.x)

                xy_cons_norm = min(distance * self.kp, VMAX)        # limit VMAX
                min_speed = self.last_speed_norm - ACCEL_MAX/RATE
                max_speed = self.last_speed_norm + ACCEL_MAX/RATE

                xy_cons_norm = clamp(min_speed, xy_cons_norm, max_speed)    # limit accel

                d_theta = normalize_angle(self.target_pos.theta - self.pos.theta)
                
                vang = clamp(-ANG_VMAX, d_theta * self.kp_ang, ANG_VMAX)
                vang = clamp(self.last_speed.vtheta - ANG_ACCEL_MAX/RATE, vang, self.last_speed.vtheta + ANG_ACCEL_MAX/RATE)

                self.last_speed = Speed(xy_cons_norm*cos(azimut), xy_cons_norm*sin(azimut), vang)
                self.last_speed_norm = xy_cons_norm

                speed_msg = hgpb.Speed(vx=self.last_speed.vx,
                                       vy=self.last_speed.vy,
                                       vtheta=self.last_speed.vtheta)
                # print(f"speed cons: {self.last_speed}")
                self.speed_pub.send(speed_msg)


    def send_speed(self, vx, vy, vtheta):
        speed_msg = hgpb.Speed(vx=vx, vy=vy, vtheta=vtheta)
        self.speed_pub.send(speed_msg)

    def on_odom_pos(self, topic, msg, timestamp):
        self.pos = Pos(msg.x, msg.y, msg.theta)
        self.last_odom_pos_time = time.time()
    
    def on_target_pos(self, topic, msg, timestamp):
        self.target_pos = Pos(msg.x, msg.y, msg.theta)
        self.loco_state = LocoState.RUNNING
        self.start_pos = self.pos
        dpos = self.target_pos - self.pos
        self.traj_duration = sqrt(dpos.x**2 + dpos.y**2 + (ROBOT_RADIUS*dpos.theta)**2) / VMAX
        self.start_time = time.time()
        print(f"set target to {self.target_pos}")


if __name__ == '__main__':
    loco = Locomotion()
    while True:
        #loco.send_speed(200, 0, 0)
        loco.update()
        time.sleep(1/RATE)
