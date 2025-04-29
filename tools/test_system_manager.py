#!/usr/bin/env python3
import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher, StringPublisher
from ecal.core.subscriber import ProtoSubscriber, StringSubscriber
from time import sleep
import sys
from enum import Enum
import generated.robot_state_pb2 as robot_pb
import generated.lidar_data_pb2 as lidar_pb
import generated.messages_pb2 as base_pb

G_NONE = 0x0
G_BASIC = 0x1

A_NONE = 0x0
A_DIRECT = 0x1
A_POS = 0x2
A_SPEED = 0x4

class SystemManager:

    def __init__(self, name="test"):
        
        self.Mode_pub = ProtoPublisher("system_modes",base_pb.System)



    def set_mode(self, asserv, guidance):
        print(asserv,guidance)
        mode = base_pb.System()
        mode.asserv = asserv
        mode.guidance = guidance
        mode.odometry = base_pb.System.OdometryFlags.ODOMETRY_ENABLED
        self.Mode_pub.send(mode)

ecal_core.initialize(sys.argv,"Test mode")
sleep(1) # laissons ecal se réveiller 
t = SystemManager()

if __name__ == "__main__":
    while (True):
        sleep(0.5)


