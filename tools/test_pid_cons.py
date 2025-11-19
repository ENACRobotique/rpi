 #!/bin/env python3
import sys
sys.path.append('../generated')
import time
import robot_state_pb2 as pb
from math import radians
import ecal.nanobind_core as ecal_core
from ecal.msg.proto.core import Publisher as ProtoPublisher


if __name__ == "__main__":
    if not ecal_core.is_initialized():
        ecal_core.initialize("position")
    cmd_pub = ProtoPublisher(pb.Position, "set_position")
    time.sleep(1)


    while True:
        cmd_msg = pb.Position(x=0,y=0,theta=radians(10))
        cmd_pub.send(cmd_msg)
        time.sleep(5)
        cmd_msg = pb.Position(x=0,y=0,theta= radians(-10))
        cmd_pub.send(cmd_msg)
        time.sleep(5)

    time.sleep(0.5)
