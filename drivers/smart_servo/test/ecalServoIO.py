#!/usr/bin/python3
import sys
import time
import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher
from actionneurs_pb2 import SmartServo

default = SmartServo.ServoType.STS


class servoIO:
  def __init__(self) -> None:
    # ecal_core.initialize(sys.argv, "smart servo test publisher")  
    self.publisher = ProtoPublisher("smart_servo", SmartServo)
    self.message = SmartServo()

  def ping(self, id, type=default ):
    self.message.id = id
    self.message.type = type
    self.message.command = SmartServo.CommandType.PING
    self.publisher.send(self.message)
  
  def setId(self, id, newID, type=default):
    self.message.id = id
    self.message.type = type
    self.message.command = SmartServo.CommandType.SET_ID
    self.message.newID = newID
    self.publisher.send(self.message)
  
  def setBaudRate(self, id,  speed, type=default):
    self.message.id = id
    self.message.type = type
    self.message.command = SmartServo.CommandType.SET_BAUDRATE
    self.message.speed = speed
    self.publisher.send(self.message)
  
  def move(self, id,  position, type=default):
    self.message.id = id
    self.message.type = type
    self.message.command = SmartServo.CommandType.MOVE
    self.message.position = position
    self.publisher.send(self.message)
  
  def moveSpeed(self, id,  position, speed, type=default):
    self.message.id = id
    self.message.type = type
    self.message.command = SmartServo.CommandType.MOVE_SPEED
    self.message.position = position
    self.message.speed = speed
    self.publisher.send(self.message)
  
  def setEndless(self, id,  status, type=default):
    self.message.id = id
    self.message.type = type
    self.message.command = SmartServo.CommandType.SET_ENDLESS
    self.message.endless_status = status
    self.publisher.send(self.message)
  
  def turn(self, id,  direction, speed, type=default):
    self.message.id = id
    self.message.type = type
    self.message.command = SmartServo.CommandType.TURN
    self.message.direction = direction
    self.message.speed = speed
    self.publisher.send(self.message)
  
  def setTorque(self, id,  torque, type=default):
    self.message.id = id
    self.message.type = type
    self.message.command = SmartServo.CommandType.SET_TORQUE
    self.message.torque = torque
    self.publisher.send(self.message)

  def enableTorque(self, id,  enable, type=default):
    self.message.id = id
    self.message.type = type
    self.message.command = SmartServo.CommandType.TORQUE_ENABLE
    self.message.enable_torque = enable
    self.publisher.send(self.message)

  def setLimits(self, id,  min, max, type=default):
    self.message.id = id
    self.message.type = type
    self.message.command = SmartServo.CommandType.SET_LIMITS
    self.message.min_angle = min
    self.message.max_angle = max
    self.publisher.send(self.message)

  def readPos(self, id, type=default):
    self.message.id = id
    self.message.type = type
    self.message.command = SmartServo.CommandType.READ_POS
    self.publisher.send(self.message)
    # TODO lire le retour


pedro = servoIO()
if __name__ == "__main__":

  while ecal_core.ok():
     time.sleep(1)

  ecal_core.finalize()