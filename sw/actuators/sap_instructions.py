#!/usr/bin/env python3
import sys
import time
import ecal.nanobind_core as ecal_core
import struct
from enum import Enum
from generated.actionneurs_pb2 import SmartServo, SAPRecord


RESP_TIMEOUT = 500



class SAPInstructions:
  def __init__(self) -> None:
    if not ecal_core.is_initialized():
      ecal_core.initialize("smart servo test publisher")
    self.client = ecal_core.ServiceClient("actuators")
    while not self.client.is_connected():
      print("Waiting for SAP service ...")
      time.sleep(1.0)

  def ping(self, id):
    rec = SAPRecord(id=id)
    response = self.client.call_with_response("ping", rec.SerializeToString(), RESP_TIMEOUT)
    if response is None:
      print("no response... service offline ?")
      return False
    msg_resp = SAPRecord()
    msg_resp.ParseFromString(response[0].response)
    if msg_resp.id == id and msg_resp.data == b'OK':
      return True
    return False
  
  def write(self, id,  reg, data):
    rec = SAPRecord(id=id, reg=reg, len=len(data), data=data)
    msg_bin = rec.SerializeToString()
    self.client.call_with_callback_async("write_reg", msg_bin, self.response_cb)

  def read(self, id, reg, len):
    rec = SAPRecord(id=id, reg=reg, len=len)
    response = self.client.call_with_response("read_reg", rec.SerializeToString(), RESP_TIMEOUT)
    if response is None:
      print("no response... service offline ?")
      return None
    msg_resp = SAPRecord()
    msg_resp.ParseFromString(response[0].response)
    if msg_resp.id == id and msg_resp.reg == reg and msg_resp.len == len and msg_resp.status == 0:
      return msg_resp.data
    return None
  
  def response_cb(self, *args):
    pass


  
  # def setId(self, id, newID, type=default):
  #   message = SmartServo()
  #   message.id = id
  #   message.type = type
  #   message.command = SmartServo.CommandType.SET_ID
  #   message.new_id = newID
  #   msg_bin = message.SerializeToString()
  #   self.client.call_with_callback_async("read_pos", msg_bin, self.response_cb)
  
  # def setBaudRate(self, id,  speed, Unlock=False, type=default):
  #   message = SmartServo()
  #   message.id = id
  #   message.type = type
  #   message.command = SmartServo.CommandType.SET_BAUDRATE
  #   message.speed = speed
  #   message.unlock_eeprom = unlock
  #   msg_bin = message.SerializeToString()
  #   self.client.call_with_callback_async("read_pos", msg_bin, self.response_cb)
  
  # def move(self, id,  position, type=default):
  #   message = SmartServo()
  #   message.id = id
  #   message.type = type
  #   message.command = SmartServo.CommandType.MOVE
  #   message.position = position
  #   msg_bin = message.SerializeToString()
  #   self.client.call_with_callback_async("read_pos", msg_bin, self.response_cb)
  
  # def moveSpeed(self, id,  position, speed, type=default):
  #   message = SmartServo()
  #   message.id = id
  #   message.type = type
  #   message.command = SmartServo.CommandType.MOVE_SPEED
  #   message.position = position
  #   message.speed = speed
  #   msg_bin = message.SerializeToString()
  #   self.client.call_with_callback_async("read_pos", msg_bin, self.response_cb)
  
  # def setEndless(self, id,  status, unlock=False, type=default):
  #   """CAUTION: This will make Multiturn servos back to turn 1 !"""
  #   message = SmartServo()
  #   message.id = id
  #   message.type = type
  #   message.command = SmartServo.CommandType.SET_ENDLESS
  #   message.endless_status = status
  #   message.unlock_eeprom = unlock
  #   msg_bin = message.SerializeToString()
  #   self.client.call_with_callback_async("read_pos", msg_bin, self.response_cb)
  
  # def turn(self, id,  direction, speed, type=default):
  #   message = SmartServo()
  #   message.id = id
  #   message.type = type
  #   message.command = SmartServo.CommandType.TURN
  #   message.direction = direction
  #   message.speed = speed
  #   msg_bin = message.SerializeToString()
  #   self.client.call_with_callback_async("read_pos", msg_bin, self.response_cb)
  
  # def setTorque(self, id,  torque, type=default):
  #   message = SmartServo()
  #   message.id = id
  #   message.type = type
  #   message.command = SmartServo.CommandType.SET_TORQUE
  #   message.torque = torque
  #   msg_bin = message.SerializeToString()
  #   self.client.call_with_callback_async("read_pos", msg_bin, self.response_cb)

  # def enableTorque(self, id,  enable, type=default):
  #   message = SmartServo()
  #   message.id = id
  #   message.type = type
  #   message.command = SmartServo.CommandType.TORQUE_ENABLE
  #   message.enable_torque = enable
  #   msg_bin = message.SerializeToString()
  #   self.client.call_with_callback_async("read_pos", msg_bin, self.response_cb)

  # def setLimits(self, id,  min, max, unlock=False, type=default):
  #   message = SmartServo()
  #   message.id = id
  #   message.type = type
  #   message.command = SmartServo.CommandType.SET_LIMITS
  #   message.min_angle = min
  #   message.max_angle = max
  #   message.unlock_eeprom = unlock
  #   msg_bin = message.SerializeToString()
  #   self.client.call_with_callback_async("read_pos", msg_bin, self.response_cb)

  # def setMultiturn(self, id, factor, unlock=False, type=default):
  #   message = SmartServo()
  #   message.id = id
  #   message.type = type
  #   message.command = SmartServo.CommandType.SET_MULTITURN
  #   message.multiturn_factor = factor
  #   message.unlock_eeprom = unlock
  #   msg_bin = message.SerializeToString()
  #   self.client.call_with_callback_async("read_pos", msg_bin, self.response_cb)

  # def readPos(self, id, type=default):
  #   message = SmartServo()
  #   message.id = id
  #   message.type = type
  #   message.command = SmartServo.CommandType.READ_POS
  #   msg_bin = message.SerializeToString()
  #   response = self.client.call_with_response("read_pos", msg_bin, 500)
  #   if (response):
  #     msg_resp = SmartServo()
  #     msg_resp.ParseFromString(response[0].response)
  #     if msg_resp.id != id:
  #       print("readPos: Response id not matching")
  #     return msg_resp.position
  #   else:
  #     print("no response...")



  # def isMoving(self, id, stype=default):
  #   message = SmartServo()
  #   message.id = id
  #   message.type = stype
  #   message.command = SmartServo.CommandType.IS_MOVING
  #   msg_bin = message.SerializeToString()
  #   response = self.client.call_with_response("read_pos", msg_bin, 500)
  #   if (response):
  #     msg_resp = SmartServo()
  #     msg_resp.ParseFromString(response[0].response)
  #     if msg_resp.id != id:
  #       print("isMoving: Response id not matching")
  #     return msg_resp.moving
  #   else:
  #     print("no response...")
      
  # def setTotalPos(self, id, total_pos: bool):
  #   if total_pos:
  #     self.write(id, 0x12, b'\x7C')
  #   else:
  #     self.write(id, 0x12, b'\x6C')


  
  # def pump(self, id, state):
  #   self.write(id, 0x20, struct.pack("<B", state))

  # def valve(self, id, state):
  #   self.write(id, 0x21, struct.pack("<B", state))

  # def valve_use(self, id):
  #   self.write(id, 0x22, struct.pack("<B", 1))

  # def id_change(self, id, state): 
  #   self.write(id, 0x02, struct.pack("<B", state))

  # def baudrate(self, id, state): 
  #   self.write(id, 0x06, struct.pack("<B", state))

  # def readCurrent(self, id):
  #   data = self.read(id, 0x23, 2)
  #   (current,) = struct.unpack("<H", data)
  #   return current 
  


if __name__ == "__main__":
  pedro = SAPBasics()
  while True:
    # a = pedro.isMoving(1)
    # b = pedro.readPos(2)
    # print(f"Droite:{a}\t Gauche:{b}\n")
    time.sleep(0.1)
