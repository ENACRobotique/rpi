 #!/bin/env python3
import sys
sys.path.append('../generated')
import time
import robot_state_pb2 as pb
from math import radians
import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher
from ecal.core.subscriber import ProtoSubscriber


if __name__ == "__main__":
    ecal_core.initialize(sys.argv, "position")
    cmd_pub = ProtoPublisher("set_position", pb.Position)
    time.sleep(1)


    while True:
        cmd_msg = pb.Position(x=0,y=0,theta=radians(10))
        cmd_pub.send(cmd_msg)
        time.sleep(5)
        cmd_msg = pb.Position(x=0,y=0,theta= radians(-10))
        cmd_pub.send(cmd_msg)
        time.sleep(5)

    time.sleep(0.5)
