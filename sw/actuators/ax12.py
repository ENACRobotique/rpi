from sap_instructions import SAPInstructions
import struct
from enum import Enum


class AX12:
    BAUDRATES = {1000000: 1, 500000:3, 400000: 4, 250000: 7, 200000:9, 115200: 16, 57600: 34, 19200: 103, 9600:207}

    def __init__(self, s:SAPInstructions) -> None:
        self.protocol = s
    
    def read_pos(self, id):
        data = self.protocol.read(id, AX12MemoryMap.CURRENT_POSITION.value, 2)
        if data is not None:
            (pos,) = struct.unpack('<H', data)
            return pos
        else :
            

    def move(self, id, pos):
        self.protocol.write(id, AX12MemoryMap.GOAL_POSITION.value, struct.pack('<H', pos))

    def move_speed(self ,id ,pos, speed):
        self.protocol.write(id, AX12MemoryMap.GOAL_POSITION.value, struct.pack('<HH', pos, speed))
    
    def set_id(self,id,new_id):
        self.protocol.write(id,AX12MemoryMap.ID.value,struct.pack('<B', new_id))
    
    def change_baudrate(self, id, baudrate):
        if baudrate in AX12.BAUDRATES:
            self.protocol.write(id,AX12MemoryMap.BAUDRATE.value,struct.pack('<B', AX12.BAUDRATES[baudrate]))
        else:
            print(f"Baurate not in list: {AX12.BAUDRATES.keys()}")
    
class AX12MemoryMap(Enum):
    """
    AX12A Memory map.
    2 bytes words are little endian.
    https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/#control-table-of-eeprom-area
    """
    ############### EEPROM ###############

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

    # Unit: 2µs. Max delay=2*254µs
    RETURN_DELAY = 5

    CW_ANGLE_LIMIT = 6   # 2 bytes
    CCW_ANGLE_LIMIT = 8  # 2 bytes

    
    TEMPERATURE_LIMIT = 11
    MIN_VOLTAGE_LIMIT = 12
    MAX_VOLTAGE_LIMIT = 13
    MAX_TORQUE = 14     # 2 bytes
    STATUS_RETURN_LEVEL = 16
    ALARM_LED = 17
    ALARM_SHUTDOWN = 18


############### RAM ###############

    TORQUE_ENABLE = 24
    LED = 25
    CW_COMPLIANCE_MARGIN = 26
    CCW_COMPLIANCE_MARGIN = 27
    CW_COMPLIANCE_SLOPE = 28
    CCW_COMPLIANCE_SLOPE = 29

    # Range: [0-1023]
    GOAL_POSITION = 30  # 2 bytes

    GOAL_SPEED = 32 # 2 bytes

    TORQUE_LIMIT = 34   # 2 bytes
    CURRENT_POSITION = 36  # 2 bytes
    CURRENT_SPEED = 38     # 2 bytes
    CUURENT_LOAD = 40 # 2 bytes
    CURRENT_VOLTAGE = 42
    CURRENT_TEMPERATURE = 43
    REGISTERED_INSTRUCTION = 44
    MOVING = 46
    LOCK = 47
    PUNCH = 48    # 2 bytes
