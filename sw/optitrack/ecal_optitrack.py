#!/usr/bin/env python3
import sys
import time
from math import pi
from NatNetClient import NatNetClient
sys.path.append('../../generated')
import robot_state_pb2 as hgpb
import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher
from ecal.core.subscriber import ProtoSubscriber
import pyquaternion

PERIOD = 0.1

def center_radians(angle):
    while angle > pi:
        angle -= 2*pi
    while angle < -pi:
        angle += 2*pi
    return angle


class OptitrackBridge:
    def __init__(self, tracked_id):
        self.tracked_id = tracked_id
        self.last_time = 0
        ecal_core.initialize(sys.argv, "Optitrack Bridge")
        self.optitrack_pos_pub = ProtoPublisher("optitrack_pos", hgpb.Position)

        self.streaming_client = NatNetClient()
        self.streaming_client.set_client_address("0.0.0.0")
        self.streaming_client.set_server_address("192.168.1.240")
        self.streaming_client.set_use_multicast(False)

        self.streaming_client.new_frame_listener = None  #receive_new_frame
        self.streaming_client.rigid_body_listener = self.receive_rigid_body_frame

        # Start up the streaming client now that the callbacks are set up.
        # This will run perpetually, and operate on a separate thread.
        self.streaming_client.run()

    # This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
    def receive_rigid_body_frame(self, new_id, position, rotation ):
        if new_id == self.tracked_id and time.time() - self.last_time >= PERIOD:
            #print( "Received frame for rigid body", new_id," ",position," ",rotation )
            px, py, pz = position
            px *= 1000
            py *= 1000
            pz *= 1000
            qi, qj, qk, qw = rotation
            quat = pyquaternion.Quaternion(qw, qi, qj, qk)
            yaw, pitch, roll = quat.yaw_pitch_roll
            theta = center_radians(yaw)

            hgm = hgpb.Position(x=px,y=py,theta=theta)
            self.optitrack_pos_pub.send(hgm)
            self.last_time = time.time()
            print(f"x:{px:.03f}  y:{py:.03f}  yaw:{theta:.03f}")

if __name__ == "__main__":
    bridge = OptitrackBridge(420)
    while True:
        time.sleep(1)
