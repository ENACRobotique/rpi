 #!/bin/env python3
import sys
sys.path.append('../../generated')
import time
import messages_pb2 as llpb

import ecal.nanobind_core as ecal_core
from ecal.msg.proto.core import Publisher as ProtoPublisher

if not ecal_core.is_initialized():
    ecal_core.initialize("set_pid")
pid_pub = ProtoPublisher(llpb.MotorPid, "pid_gains")

def set_pids(nb, kp, ki, kd):
    msg = llpb.MotorPid(motor_no=nb, kp=kp, ki=ki, kd=kd)
    pid_pub.send(msg)
