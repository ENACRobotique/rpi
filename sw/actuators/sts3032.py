from sap_instructions import SAPInstructions
import struct
from enum import Enum


class STS3032:
    def __init__(self, s:SAPInstructions) -> None:
        self.protocol = s
    
    def readPos(self, id):
        data = self.protocol.read(id, STSMemoryMap.CURRENT_POSITION.value, 2)
        if data is not None:
            (pos,) = struct.unpack('<H', data)
            return pos
    
    def lock(self, id, lock):
        self.protocol.write(id, STSMemoryMap.EEPROM_LOCK.value, struct.pack('>B', lock))
    
    def move(self, id, pos):
        self.protocol.write(id, STSMemoryMap.GOAL_POSITION.value, struct.pack('<H', pos))
        
    def move_speed(self, id, pos, speed):
        self.protocol.write(id, STSMemoryMap.GOAL_POSITION.value, struct.pack('<HHH', pos,1000, speed))
    

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
