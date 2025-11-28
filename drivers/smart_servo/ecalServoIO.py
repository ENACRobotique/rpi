#!/usr/bin/env python3
import sys
import time
import ecal.nanobind_core as ecal_core
from queue import Queue, Empty

sys.path.append("../../")

from generated.actionneurs_pb2 import SmartServo, SAPRecord


RESP_TIMEOUT = 0.1

default = SmartServo.ServoType.STS
AX12 = SmartServo.ServoType.AX12

class servoIO:
  def __init__(self) -> None:
    if not ecal_core.is_initialized():
      ecal_core.initialize("smart servo test publisher")
    self.client = ecal_core.ServiceClient("actuators")
    self.message = SmartServo()

  
  def response_cb(self, *args):
    pass
    


  def ping(self, id, type=default ):
    self.message.id = id
    self.message.type = type
    self.message.command = SmartServo.CommandType.PING
    msg_bin = self.message.SerializeToString()
    self.client.call_with_callback_async("read_pos", msg_bin, self.response_cb)
  
  def setId(self, id, newID, type=default):
    self.message.id = id
    self.message.type = type
    self.message.command = SmartServo.CommandType.SET_ID
    self.message.new_id = newID
    msg_bin = self.message.SerializeToString()
    self.client.call_with_callback_async("read_pos", msg_bin, self.response_cb)
  
  def setBaudRate(self, id,  speed, type=default):
    self.message.id = id
    self.message.type = type
    self.message.command = SmartServo.CommandType.SET_BAUDRATE
    self.message.speed = speed
    msg_bin = self.message.SerializeToString()
    self.client.call_with_callback_async("read_pos", msg_bin, self.response_cb)
  
  def move(self, id,  position, type=default):
    self.message.id = id
    self.message.type = type
    self.message.command = SmartServo.CommandType.MOVE
    self.message.position = position
    msg_bin = self.message.SerializeToString()
    self.client.call_with_callback_async("read_pos", msg_bin, self.response_cb)
  
  def moveSpeed(self, id,  position, speed, type=default):
    self.message.id = id
    self.message.type = type
    self.message.command = SmartServo.CommandType.MOVE_SPEED
    self.message.position = position
    self.message.speed = speed
    msg_bin = self.message.SerializeToString()
    self.client.call_with_callback_async("read_pos", msg_bin, self.response_cb)
  
  def setEndless(self, id,  status, type=default):
    """CAUTION: This will make Multiturn servos back to turn 1 !"""
    self.message.id = id
    self.message.type = type
    self.message.command = SmartServo.CommandType.SET_ENDLESS
    self.message.endless_status = status
    msg_bin = self.message.SerializeToString()
    self.client.call_with_callback_async("read_pos", msg_bin, self.response_cb)
  
  def turn(self, id,  direction, speed, type=default):
    self.message.id = id
    self.message.type = type
    self.message.command = SmartServo.CommandType.TURN
    self.message.direction = direction
    self.message.speed = speed
    msg_bin = self.message.SerializeToString()
    self.client.call_with_callback_async("read_pos", msg_bin, self.response_cb)
  
  def setTorque(self, id,  torque, type=default):
    self.message.id = id
    self.message.type = type
    self.message.command = SmartServo.CommandType.SET_TORQUE
    self.message.torque = torque
    msg_bin = self.message.SerializeToString()
    self.client.call_with_callback_async("read_pos", msg_bin, self.response_cb)

  def enableTorque(self, id,  enable, type=default):
    self.message.id = id
    self.message.type = type
    self.message.command = SmartServo.CommandType.TORQUE_ENABLE
    self.message.enable_torque = enable
    msg_bin = self.message.SerializeToString()
    self.client.call_with_callback_async("read_pos", msg_bin, self.response_cb)

  def setLimits(self, id,  min, max, type=default):
    self.message.id = id
    self.message.type = type
    self.message.command = SmartServo.CommandType.SET_LIMITS
    self.message.min_angle = min
    self.message.max_angle = max
    msg_bin = self.message.SerializeToString()
    self.client.call_with_callback_async("read_pos", msg_bin, self.response_cb)

  def setMultiturn(self, id, factor, unlock=False, type=default):
    self.message.id = id
    self.message.type = type
    self.message.command = SmartServo.CommandType.SET_MULTITURN
    self.message.multiturn_factor = factor
    self.message.unlock_eeprom = unlock
    msg_bin = self.message.SerializeToString()
    self.client.call_with_callback_async("read_pos", msg_bin, self.response_cb)

  def readPos(self, id, type=default):
    self.message.id = id
    self.message.type = type
    self.message.command = SmartServo.CommandType.READ_POS
    msg_bin = self.message.SerializeToString()
    response = self.client.call_with_response("read_pos", msg_bin, 500)
    if (response):
      msg_resp = SmartServo()
      msg_resp.ParseFromString(response[0].response)
      if msg_resp.id != id:
        print("readPos: Response id not matching")
      return msg_resp.position
    else:
      print("no response...")



  def isMoving(self, id, stype=default):
    self.message.id = id
    self.message.type = stype
    self.message.command = SmartServo.CommandType.IS_MOVING
    msg_bin = self.message.SerializeToString()
    response = self.client.call_with_response("read_pos", msg_bin, 500)
    if (response):
      msg_resp = SmartServo()
      msg_resp.ParseFromString(response[0].response)
      if msg_resp.id != id:
        print("isMoving: Response id not matching")
      return msg_resp.moving
    else:
      print("no response...")
      
  def setTotalPos(self, id, total_pos: bool):
    if total_pos:
      self.write(id, 0x12, b'\x7C')
    else:
      self.write(id, 0x12, b'\x6C')

  def write(self, id,  reg, data, type=default):
    rec = SAPRecord(id=id, reg=reg, len=len(data), data=data)
    msg_bin = rec.SerializeToString()
    self.client.call_with_callback_async("write_reg", msg_bin, self.response_cb)

  def read(self, id, reg, len):
    rec = SAPRecord(id=id, reg=reg, len=len)

    response = self.client.call_with_response("read_reg", rec.SerializeToString(), 500)
    if (response):
      msg_resp = SAPRecord()
      msg_resp.ParseFromString(response[0].response)
      if msg_resp.id == id and msg_resp.reg == reg and msg_resp.len == len:
        return msg_resp.data
    else:
      print("no response...")



if __name__ == "__main__":
  pedro = servoIO()
  while True:
    a = pedro.isMoving(1)
    b = pedro.readPos(2)
    print(f"Droite:{a}\t Gauche:{b}\n")
    time.sleep(0.1)
