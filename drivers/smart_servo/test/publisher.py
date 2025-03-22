#!/usr/bin/python3
import sys
import time
import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher
from actionneurs_pb2 import SmartServo

if __name__ == "__main__":

  ecal_core.initialize(sys.argv, "smart servo test publisher")

  pub = ProtoPublisher("smart_servo", SmartServo)
  message = SmartServo()
  while ecal_core.ok():
    message.id = 1
    message.type = SmartServo.ServoType.STS
    message.command = SmartServo.CommandType.PING
    pub.send(message)
    time.sleep(1)
# finalize eCAL API

  ecal_core.finalize()