 #!/bin/env python3
import sys
sys.path.append('../../generated')
import time
import messages_pb2 as llpb

import ecal.nanobind_core as ecal_core
from ecal.msg.proto.core import Publisher as ProtoPublisher


if __name__ == "__main__":
    if not ecal_core.is_initialized():
        ecal_core.initialize("set_pid")
    pid_pub = ProtoPublisher(llpb.MotorPid, "pid_gains")
    time.sleep(1)
    pid_msg = llpb.MotorPid()

    if len(sys.argv) > 3:
        pid_msg.kp = float(sys.argv[1])
        pid_msg.ki = float(sys.argv[2])
        pid_msg.kd = float(sys.argv[3])
        pid_pub.send(pid_msg)
    
    time.sleep(0.5)
