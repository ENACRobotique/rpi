from sap_instructions import SAPInstructions
import struct
from enum import Enum


class Pump:
    def __init__(self, s:SAPInstructions) -> None:
        self.protocol = s
    
    def pump(self, id, pump):
        self.protocol.write(id, PumpMemoryMap.PUMP.value, struct.pack('>B', pump))
    



class PumpMemoryMap(Enum):
    """
    Pump Memory map.
    2 bytes words are little endian.
    """
    
    # Constant value: 20
    MODEL_NUMBER = 0 # 2 bytes
    
    ID = 2
    
    # Unit: 2µs. Max delay=2*254µs
    RETURN_DELAY = 0x05
  
    # 1: 1000000
    # 3: 500000
    # 4: 400000
    # 7: 250000
    # 9: 200000
    # 16: 115200
    # 34: 57600
    # 103: 19200
    # 207: 9600
    BAUDRATE = 0x06
    
    # Unit: percent. Range: [0-100]
    PUMP_DUTY = 0x07
    
    # Unit: percent. Range: [0-100]
    VALVE_DUTY = 0x08
    
    # Unit: milliseconds
    VALVE_RELEASE_TIME = 0x09
    
    # 0: turn off, 1: turn on
    PUMP = 0x20
    
    # 0: close, 1: open
    VALVE = 0x21
    
    # write anything to open the valse during VALVE_RELEASE_TIME, then close it
    VALVE_USE = 0x22
    
    CURRENT = 0x23  # 2 bytes, little endian
    
