#!/usr/bin/python3
import sys
import time
import ecal.core.core as ecal_core
import ecal.core.service as ecal_service
from ecal.core.publisher import ProtoPublisher
from queue import Queue, Empty

sys.path.append("../../../")

from generated.actionneurs_pb2 import SmartServo


RESP_TIMEOUT = 0.1

default = SmartServo.ServoType.STS
AX12 = SmartServo.ServoType.AX12

class servoIO:
  def __init__(self) -> None:
    if not ecal_core.is_initialized():
      ecal_core.initialize(sys.argv, "smart servo test publisher")  
    self.publisher = ProtoPublisher("smart_servo", SmartServo)
    self.client = ecal_service.Client("actuators")
    self.client.add_response_callback(self.response_cb)
    self.q = Queue()
    self.message = SmartServo()
    self.z_init1 = 0
    self.z_init2 = 0
    self.delta1H = 3850
    self.delta2H = 3800
  
  def response_cb(self, service_info, response):
    msg_resp = SmartServo()
    msg_resp.ParseFromString(response)

    if msg_resp.command == SmartServo.CommandType.READ_POS:
      self.q.put(msg_resp)
    


  def ping(self, id, type=default ):
    self.message.id = id
    self.message.type = type
    self.message.command = SmartServo.CommandType.PING
    msg_bin = self.message.SerializeToString()
    self.client.call_method("read_pos", msg_bin)
  
  def setId(self, id, newID, type=default):
    self.message.id = id
    self.message.type = type
    self.message.command = SmartServo.CommandType.SET_ID
    self.message.newID = newID
    msg_bin = self.message.SerializeToString()
    self.client.call_method("read_pos", msg_bin)
  
  def setBaudRate(self, id,  speed, type=default):
    self.message.id = id
    self.message.type = type
    self.message.command = SmartServo.CommandType.SET_BAUDRATE
    self.message.speed = speed
    msg_bin = self.message.SerializeToString()
    self.client.call_method("read_pos", msg_bin)
  
  def move(self, id,  position, type=default):
    self.message.id = id
    self.message.type = type
    self.message.command = SmartServo.CommandType.MOVE
    self.message.position = position
    msg_bin = self.message.SerializeToString()
    self.client.call_method("read_pos", msg_bin)
  
  def moveSpeed(self, id,  position, speed, type=default):
    self.message.id = id
    self.message.type = type
    self.message.command = SmartServo.CommandType.MOVE_SPEED
    self.message.position = position
    self.message.speed = speed
    msg_bin = self.message.SerializeToString()
    self.client.call_method("read_pos", msg_bin)
  
  def setEndless(self, id,  status, type=default):
    self.message.id = id
    self.message.type = type
    self.message.command = SmartServo.CommandType.SET_ENDLESS
    self.message.endless_status = status
    msg_bin = self.message.SerializeToString()
    self.client.call_method("read_pos", msg_bin)
  
  def turn(self, id,  direction, speed, type=default):
    self.message.id = id
    self.message.type = type
    self.message.command = SmartServo.CommandType.TURN
    self.message.direction = direction
    self.message.speed = speed
    msg_bin = self.message.SerializeToString()
    self.client.call_method("read_pos", msg_bin)
  
  def setTorque(self, id,  torque, type=default):
    self.message.id = id
    self.message.type = type
    self.message.command = SmartServo.CommandType.SET_TORQUE
    self.message.torque = torque
    msg_bin = self.message.SerializeToString()
    self.client.call_method("read_pos", msg_bin)

  def enableTorque(self, id,  enable, type=default):
    self.message.id = id
    self.message.type = type
    self.message.command = SmartServo.CommandType.TORQUE_ENABLE
    self.message.enable_torque = enable
    msg_bin = self.message.SerializeToString()
    self.client.call_method("read_pos", msg_bin)

  def setLimits(self, id,  min, max, type=default):
    self.message.id = id
    self.message.type = type
    self.message.command = SmartServo.CommandType.SET_LIMITS
    self.message.min_angle = min
    self.message.max_angle = max
    msg_bin = self.message.SerializeToString()
    self.client.call_method("read_pos", msg_bin)

  def setMultiturn(self, id, factor, unlock=False, type=default):
    self.message.id = id
    self.message.type = type
    self.message.command = SmartServo.CommandType.SET_MULTITURN
    self.message.multiturn_factor = factor
    self.message.unlock_eeprom = unlock
    msg_bin = self.message.SerializeToString()
    self.client.call_method("read_pos", msg_bin)

  def readPos(self, id, type=default):
    self.message.id = id
    self.message.type = type
    self.message.command = SmartServo.CommandType.READ_POS
    msg_bin = self.message.SerializeToString()
    self.client.call_method("read_pos", msg_bin)
    try:
      msg_resp = self.q.get(timeout=RESP_TIMEOUT)
      if msg_resp.id == id:
        return msg_resp.position
    except Empty:
      print("no response...")
  
  def lift(self,id, up:bool):
    if up : 
      self.move(1,self.z_init1)
      self.move(2,self.z_init2+ self.delta2H)
    else : 
      self.move(1,self.z_init1 + self.delta1H)
      self.move(2,self.z_init2)

  def set_init(self,z1,z2):
    self.z_init1 = z1
    self.z_init2 = z2

  