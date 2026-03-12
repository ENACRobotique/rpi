from sap_instructions import SAPInstructions
import struct
from enum import Enum


class AX12:
    def __init__(self, s:SAPInstructions) -> None:
        self.protocol = s

    def move(self, id, pos):
        self.protocol.write(id, AX12MemoryMap.GOAL_POSITION.value, struct.pack('<H', pos))
    




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

