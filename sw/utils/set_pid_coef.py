 #!/bin/env python3
import sys
sys.path.append('../../generated')
import time
import messages_pb2 as llpb

import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher
from ecal.core.subscriber import ProtoSubscriber


if __name__ == "__main__":
    ecal_core.initialize(sys.argv, "set_pid")
    pid_pub = ProtoPublisher("pid_gains", llpb.MotorPid)
    time.sleep(1)
    pid_msg = llpb.MotorPid()

    if len(sys.argv) > 3:
        pid_msg.kp = float(sys.argv[1])
        pid_msg.ki = float(sys.argv[2])
        pid_msg.kd = float(sys.argv[3])
        pid_pub.send(pid_msg)
    
    time.sleep(0.5)
