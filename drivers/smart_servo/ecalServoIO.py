#!/usr/bin/python3
import sys
import time
import ecal.core.core as ecal_core
import ecal.core.service as ecal_service
from ecal.core.publisher import ProtoPublisher
from queue import Queue, Empty

sys.path.append("../../")

from generated.actionneurs_pb2 import SmartServo, SAPRecord


RESP_TIMEOUT = 0.1

default = SmartServo.ServoType.STS
AX12 = SmartServo.ServoType.AX12

class servoIO:
  def __init__(self) -> None:
    if not ecal_core.is_initialized():
      ecal_core.initialize(sys.argv, "smart servo test publisher")
    self.client = ecal_service.Client("actuators")
    self.client.add_response_callback(self.response_cb)
    self.q = Queue()
    self.message = SmartServo()
    self.q_readreg = Queue()

  
  def response_cb(self, service_info, response):
    if service_info["method_name"]=="write_reg":
      pass
    elif service_info["method_name"]=="read_reg":
      msg_resp = SAPRecord()
      msg_resp.ParseFromString(response)
      self.q_readreg.put(msg_resp)
    else:
      msg_resp = SmartServo()
      msg_resp.ParseFromString(response)

      if msg_resp.command == SmartServo.CommandType.READ_POS or \
         msg_resp.command == SmartServo.CommandType.IS_MOVING:
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
    self.message.new_id = newID
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
    """CAUTION: This will make Multiturn servos back to turn 1 !"""
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

  def isMoving(self, id, type=default):
    self.message.id = id
    self.message.type = type
    self.message.command = SmartServo.CommandType.IS_MOVING
    msg_bin = self.message.SerializeToString()
    self.client.call_method("read_pos", msg_bin)
    try:
      msg_resp = self.q.get(timeout=RESP_TIMEOUT)
      if msg_resp.id == id:
        return msg_resp.moving
    except Empty:
      print("no response...")
      
  def setTotalPos(self, id, total_pos: bool):
    if total_pos:
      self.write(id, 0x12, b'\x7C')
    else:
      self.write(id, 0x12, b'\x6C')

  def write(self, id,  reg, data, type=default):
    rec = SAPRecord(id=id, reg=reg, len=len(data), data=data)
    msg_bin = rec.SerializeToString()
    self.client.call_method("write_reg", msg_bin)

  def read(self, id, reg, len):
    rec = SAPRecord(id=id, reg=reg, len=len)
    self.client.call_method("read_reg", rec.SerializeToString())
    try:
      msg_resp = self.q_readreg.get(timeout=RESP_TIMEOUT)
      if msg_resp.id == id and msg_resp.reg == reg and msg_resp.len == len:
        return msg_resp.data
    except Empty:
      print("no response...")



if __name__ == "__main__":
  pedro = servoIO()
  while True:
    a = pedro.readPos(1)
    b = pedro.readPos(2)
    print(f"Droite:{a}\t Gauche:{b}\n")
    time.sleep(0.1)
