from sap_instructions import SAPInstructions
import struct
from enum import Enum


class SCS0009:
    def __init__(self, s:SAPInstructions) -> None:
        self.protocol = s
        
    def readPos(self, id):
        data = self.protocol.read(id, SCSMemoryMap.CURRENT_POSITION.value, 2)
        if data is not None:
            (pos,) = struct.unpack('>H', data)
            return pos

    def move(self, id, pos):
        self.protocol.write(id, SCSMemoryMap.GOAL_POSITION.value, struct.pack('>H', pos))
    




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

