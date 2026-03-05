#!/usr/bin/env python3
import sys
import time
import ecal.nanobind_core as ecal_core
from queue import Queue, Empty
import struct
from enum import Enum

sys.path.append("../../")

from generated.actionneurs_pb2 import SmartServo, SAPRecord


RESP_TIMEOUT = 0.1

default = SmartServo.ServoType.STS
AX12 = SmartServo.ServoType.AX12


class ActuatorType(Enum):
  STS3032 = 0
  AX12 = 1
  SCS009 = 2
  PUMP = 3


class servoIO:
  def __init__(self) -> None:
    if not ecal_core.is_initialized():
      ecal_core.initialize("smart servo test publisher")
    self.client = ecal_core.ServiceClient("actuators")
    
    # associate IDs with actuator type
    self.types_id: dict[int, ActuatorType] = {}

  
  def response_cb(self, *args):
    pass
  
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

  def _moveSTS(self, id, pos):
        self.write(id, STSMemoryMap.GOAL_POSITION, struct.pack('<H', pos))
  
  def _moveAX12(self, id, pos):
        self.write(id, AX12MemoryMap.GOAL_POSITION, struct.pack('<H', pos))

  def _moveSCS(self, id, pos):
      self.write(id, SCSMemoryMap.GOAL_POSITION, struct.pack('>H', pos))




##############################################################################
########## OLD code, to be removed when the new version is finished ##########
##############################################################################


  def ping(self, id, type=default ):
    message = SmartServo()
    message.id = id
    message.type = type
    message.command = SmartServo.CommandType.PING
    msg_bin = message.SerializeToString()
    self.client.call_with_callback_async("read_pos", msg_bin, self.response_cb)
  
  def setId(self, id, newID, type=default):
    message = SmartServo()
    message.id = id
    message.type = type
    message.command = SmartServo.CommandType.SET_ID
    message.new_id = newID
    msg_bin = message.SerializeToString()
    self.client.call_with_callback_async("read_pos", msg_bin, self.response_cb)
  
  def setBaudRate(self, id,  speed, type=default):
    message = SmartServo()
    message.id = id
    message.type = type
    message.command = SmartServo.CommandType.SET_BAUDRATE
    message.speed = speed
    msg_bin = message.SerializeToString()
    self.client.call_with_callback_async("read_pos", msg_bin, self.response_cb)
  
  def move(self, id,  position, type=default):
    message = SmartServo()
    message.id = id
    message.type = type
    message.command = SmartServo.CommandType.MOVE
    message.position = position
    msg_bin = message.SerializeToString()
    self.client.call_with_callback_async("read_pos", msg_bin, self.response_cb)
  
  def moveSpeed(self, id,  position, speed, type=default):
    message = SmartServo()
    message.id = id
    message.type = type
    message.command = SmartServo.CommandType.MOVE_SPEED
    message.position = position
    message.speed = speed
    msg_bin = message.SerializeToString()
    self.client.call_with_callback_async("read_pos", msg_bin, self.response_cb)
  
  def setEndless(self, id,  status, type=default):
    """CAUTION: This will make Multiturn servos back to turn 1 !"""
    message = SmartServo()
    message.id = id
    message.type = type
    message.command = SmartServo.CommandType.SET_ENDLESS
    message.endless_status = status
    msg_bin = message.SerializeToString()
    self.client.call_with_callback_async("read_pos", msg_bin, self.response_cb)
  
  def turn(self, id,  direction, speed, type=default):
    message = SmartServo()
    message.id = id
    message.type = type
    message.command = SmartServo.CommandType.TURN
    message.direction = direction
    message.speed = speed
    msg_bin = message.SerializeToString()
    self.client.call_with_callback_async("read_pos", msg_bin, self.response_cb)
  
  def setTorque(self, id,  torque, type=default):
    message = SmartServo()
    message.id = id
    message.type = type
    message.command = SmartServo.CommandType.SET_TORQUE
    message.torque = torque
    msg_bin = message.SerializeToString()
    self.client.call_with_callback_async("read_pos", msg_bin, self.response_cb)

  def enableTorque(self, id,  enable, type=default):
    message = SmartServo()
    message.id = id
    message.type = type
    message.command = SmartServo.CommandType.TORQUE_ENABLE
    message.enable_torque = enable
    msg_bin = message.SerializeToString()
    self.client.call_with_callback_async("read_pos", msg_bin, self.response_cb)

  def setLimits(self, id,  min, max, type=default):
    message = SmartServo()
    message.id = id
    message.type = type
    message.command = SmartServo.CommandType.SET_LIMITS
    message.min_angle = min
    message.max_angle = max
    msg_bin = message.SerializeToString()
    self.client.call_with_callback_async("read_pos", msg_bin, self.response_cb)

  def setMultiturn(self, id, factor, unlock=False, type=default):
    message = SmartServo()
    message.id = id
    message.type = type
    message.command = SmartServo.CommandType.SET_MULTITURN
    message.multiturn_factor = factor
    message.unlock_eeprom = unlock
    msg_bin = message.SerializeToString()
    self.client.call_with_callback_async("read_pos", msg_bin, self.response_cb)

  def readPos(self, id, type=default):
    message = SmartServo()
    message.id = id
    message.type = type
    message.command = SmartServo.CommandType.READ_POS
    msg_bin = message.SerializeToString()
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
    message = SmartServo()
    message.id = id
    message.type = stype
    message.command = SmartServo.CommandType.IS_MOVING
    msg_bin = message.SerializeToString()
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


  
  def pump(self, id, state):
    self.write(id, 0x20, struct.pack("<B", state))

  def valve(self, id, state):
    self.write(id, 0x21, struct.pack("<B", state))

  def valve_use(self, id):
    self.write(id, 0x22, struct.pack("<B", 1))

  def id_change(self, id, state): 
    self.write(id, 0x02, struct.pack("<B", state))

  def baudrate(self, id, state): 
    self.write(id, 0x06, struct.pack("<B", state))

  def readCurrent(self, id):
    data = self.read(id, 0x23, 2)
    (current,) = struct.unpack("<H", data)
    return current 
  

##############################################################################
####################             END OF OLD CODE          ####################
##############################################################################



##############################################################################
####################             MEMORY TABLES            ####################
##############################################################################


class STSMemoryMap(Enum):
    """
    STS3032 Memory map.
    2 bytes words are little endian.
    """
    FIRMWARE_MAJOR_VERSION = 0x00
    FIRMWARE_MINOR_VERSION = 0x01
    SERVO_MAJOR_VERSION = 0x03
    SERVO_MINOR_VERSION = 0x04
    ID = 0x05

    # 0: 1000000
    # 1: 500000
    # 2: 250000
    # 3: 128000
    # 4: 115200
    # 5: 76800
    # 6: 57600
    # 7: 38400
    BAUDRATE = 0x06

    # Unit: 2µs. Max delay=2*254µs
    RETURN_DELAY = 0x07

    # 0: respond only for PING and READ
    # 1: always respond
    RESPONSE_STATUS_LEVEL = 0x08

    # Range: [0-4094]
    MIN_ANGLE_LIMIT = 0x09  # 2 bytes
    
    # Range: [0-4095]
    MAX_ANGLE_LIMIT = 0x0B    # 2 bytes
    
    # Max temp limit, in °C. Range: [0-100]
    MAX_TEMP = 0x0D
    
    # Unit: 0.1V Range:[0-254]
    MAX_INPUT_VOLTAGE = 0x0E
    
    # Unit: 0.1V Range:[0-254]
    MIN_INPUT_VOLTAGE = 0x0F

    # Unit: 0.1% Range:[0-1000]
    MAX_TORQUE = 0x10   # 2 bytes

    # bit-mask:
    # 0: 0/1: forward/reverse coefficient ???
    # 1: 0/1: brushless/brushed motor
    # 2: 0/1: 50/1 steps/s
    # 3: 0/1: speed 0 stop / max speed
    # 4: 0/1: feedback lap/full angle
    # 5: 0/1: 1.5K low voltage sampling / 1K high voltage sampling
    # 6: 0/1: High frequency without dead zone /  dead zone at low frequency
    # 7: 0/1: positive / reverse direction
    PHASE = 0x12

    # bit-mask, set bit to 1 to enable corresponding protection
    # 0: voltage
    # 1: magnetic coding protection ?
    # 2: overheating
    # 3: overcurrent
    # 5: load overload
    UNLOADING_CONDITION = 0x13
    # same as UNLOADING_CONDITION
    LED_ALARM_CONDITION = 0x14

    PID_P = 0x15
    PID_D = 0x16
    PID_I = 0x17    # NOT DEFINED ?

    # Minimum starting output torque. Unit: 0.1% Range:[0-1000]
    MIN_STARTING_TORQUE = 0x18  # 2 bytes

    # Max integral
    # Range: [0-254]
    INTEGRAL_LIMIT_VALUE = 0x19

    # insensitive area
    # Range: [0-32]
    CW_DEAD = 0x1A
    CCW_DEAD = 0x1B

    # Protection current
    # Unit: 6.5mA. Range: [0-500]
    PROTECTION_CURRENT = 0x1C   # 2 bytes

    # Angle resolution. Set bit 4 of PHASE register to 1 when using this.
    ANGLE_RESOLUTION = 0x1E

    # Offset angle.
    # bit 11 is the direction
    # bits[10:0] in the range 0-2047
    POSITION_CORRECTION = 0x1F  # 2 bytes

    # 0: position servo mode
    # 1: motor constant speed mode, controlled by GOAL_SPEED, bit 15 is direction bit.
    # 2: PWM open-loop speed mode, controlled by GOAL_TIME, bit 10 is direction bit
    # 3: stepper servo mode, controlled by GOAL_POSITION, bit 15 is direction bit
    OPERATING_MODE = 0x21

    # Torque after overload protection triggered. Unit: % of max torque. Range: [0-100]
    PROTECTION_TORQUE = 0x22
    
    # Unit: 10ms Range:[0-254]
    PROTECTION_TIME = 0x23

    # Torque at which the timing of overload protection starts
    # Unit: % of max torque. Range: [0-100]
    OVERLOAD_TORQUE = 0x24

    # speed PID on motor constant speed mode (OPERATING_MODE:1)
    SPEED_PID_P = 0x25
    OVERCURRENT_PROTECTION = 0x26
    SPEED_PID_I = 0x27

    # 0: disable torque output
    # 1: torque enabled
    # 128: Any current position is more positive than 2048 ???
    TORQUE_SWITCH = 0x28

    # Unit: 100 step/s². Range: [0-254]
    ACCELERATION = 0x29

    # Range: [-32766 - 32766]
    GOAL_POSITION = 0x2A # 2 bytes

    # In PWM open-loop speed mode (OPERATING_MODE:3), Range: [50-1000]. Bit 10 is direction bit
    GOAL_TIME = 0x2C    # 2 bytes

    # Unit: step/s. Range: [-32766 - 32766]
    GOAL_SPEED = 0x2E   # 2 bytes

    # Unit: 0.1% Range: [0-1000] or 1% [0-100] ???
    TORQUE_LIMIT = 0x30     # 2 bytes
    
    # EEPROM Lock. 0: Unlock, 1: Lock
    EEPROM_LOCK = 0x37

    CURRENT_POSITION = 0x38 # 2 bytes
    
    # Unit: steps/s
    CURRENT_SPEED = 0x3A    # 2 bytes
    
    # Unit: 0.1%
    CURRENT_LOAD = 0x3C     # 2 bytes

    # Unit: 0.1V
    CURRENT_VOLTAGE = 0x3E
    
    # Unit: °C
    CURRENT_TEMPERATURE = 0x3F
    
    # 1 when using registered actions
    ASYNCHRONOUS_WRITE = 0x40
    
    # servo status bit-mask.
    # a bit set to 1 indicates the corresponding error occured
    SERVO_STATUS = 0x41
    
    # 1 when servo in motion, 0 when stopped
    MOVING = 0x42

    # Unit: 6.5mA
    CURRENT_CURRENT = 0x45



class SCSMemoryMap(Enum):
    """
    SCS0009 Memory map.
    2 bytes words are big endian.
    """
    FIRMWARE_MAJOR_VERSION = 0x00
    FIRMWARE_MINOR_VERSION = 0x01
    SERVO_MAJOR_VERSION = 0x03
    SERVO_MINOR_VERSION = 0x04
    ID = 0x05

    # 0: 1000000
    # 1: 500000
    # 2: 250000
    # 3: 128000
    # 4: 115200
    # 5: 76800
    # 6: 57600
    # 7: 38400
    BAUDRATE = 0x06

    # Unit: 2µs. Max delay=2*254µs
    RETURN_DELAY = 0x07

    # 0: respond only for PING and READ
    # 1: always respond
    RESPONSE_STATUS_LEVEL = 0x08

    # Range: [0-1022]
    MIN_ANGLE_LIMIT = 0x09  # 2 bytes
    
    # Range: [0-1023]
    MAX_ANGLE_LIMIT = 0x0B    # 2 bytes
    
    # Max temp limit, in °C. Range: [0-100]
    MAX_TEMP = 0x0D
    
    # Unit: 0.1V Range:[0-254]
    MAX_INPUT_VOLTAGE = 0x0E
    
    # Unit: 0.1V Range:[0-254]
    MIN_INPUT_VOLTAGE = 0x0F

    # Unit: 0.1% Range:[0-1000]
    MAX_TORQUE = 0x10   # 2 bytes

    # ???
    PHASE = 0x12

    # bit-mask, set bit to 1 to enable corresponding protection
    # 0: voltage
    # 1: magnetic coding protection ?
    # 2: overheating
    # 3: overcurrent
    # 5: load overload
    UNLOADING_CONDITION = 0x13
    # same as UNLOADING_CONDITION
    LED_ALARM_CONDITION = 0x14

    PID_P = 0x15
    PID_D = 0x16
    # PID_I = 0x17    # NOT DEFINED ?

    # Minimum starting output torque. Unit: 0.1% Range:[0-1000]
    MIN_STARTING_TORQUE = 0x18  # 2 bytes
    
    # insensitive area
    # Range: [0-32]
    CW_DEAD = 0x1A
    CCW_DEAD = 0x1B

    # Torque after overload protection triggered. Unit: % of max torque. Range: [0-100]
    PROTECTION_TORQUE = 0x25
    
    # Unit: 40ms Range:[0-254]
    PROTECTION_TIME = 0x26

    # Torque at which the timing of overload protection starts
    # Unit: % of max torque. Range: [0-100]
    OVERLOAD_TORQUE = 0x27

    # 0: disable torque output
    # 1: torque enabled
    # 2: damping state
    TORQUE_SWITCH = 0x28

    # Range: [0-1023] or more like [0-1000] ?
    GOAL_POSITION = 0x2A # 2 bytes

    # movement time from current to goal position. Used when SPEED = 0
    # Unit: 1ms. Range: [0-9999]
    GOAL_TIME = 0x2C    # 2 bytes

    # Unit: step/s. Range: [0-1000]
    GOAL_SPEED = 0x2E   # 2 bytes
    
    # EEPROM Lock. 0: Unlock, 1: Lock
    EEPROM_LOCK = 0x30

    CURRENT_POSITION = 0x38 # 2 bytes
    
    # Unit: steps/s
    CURRENT_SPEED = 0x3A    # 2 bytes
    
    # Unit: 0.1%
    CURRENT_LOAD = 0x3C     # 2 bytes

    # Unit: 0.1V
    CURRENT_VOLTAGE = 0x3E
    
    # Unit: °C
    CURRENT_TEMPERATURE = 0x3F
    
    # 1 when using registered actions
    ASYNCHRONOUS_WRITE = 0x40
    
    # servo status bit-mask.
    # a bit set to 1 indicates the corresponding error occured
    SERVO_STATUS = 0x41
    
    # 1 when servo in motion, 0 when stopped
    MOVING = 0x42


class AX12MemoryMap(Enum):
    """
    AX12A Memory map.
    2 bytes words are little endian.
    """
    MODEL_NUMBER = 0 # 2 bytes
    FIRMWARE_VERSION = 2
    ID = 3
    
    # 1: 1000000
    # 3: 500000
    # 4: 400000
    # 7: 250000
    # 9: 200000
    # 16: 115200
    # 34: 57600
    # 103: 19200
    # 207: 9600
    BAUDRATE = 4

    # TODO, see: https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/#control-table-of-eeprom-area

    # Range: [0-1023]
    GOAL_POSITION = 30  # 2 bytes


class PumpMemoryTable(Enum):
    """
    Pump Memory map.
    2 bytes words are little endian.
    """
    ID = 3
  
    # 1: 1000000
    # 3: 500000
    # 4: 400000
    # 7: 250000
    # 9: 200000
    # 16: 115200
    # 34: 57600
    # 103: 19200
    # 207: 9600
    BAUDRATE = 4
    PUMP = 0x20
    VALVE = 0x21
    

if __name__ == "__main__":
  pedro = servoIO()
  while True:
    a = pedro.isMoving(1)
    b = pedro.readPos(2)
    print(f"Droite:{a}\t Gauche:{b}\n")
    time.sleep(0.1)
