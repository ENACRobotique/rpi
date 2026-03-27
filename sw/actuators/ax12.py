from sap_instructions import SAPInstructions
import struct
from enum import Enum


class AX12:
    def __init__(self, s:SAPInstructions) -> None:
        self.protocol = s

    def move(self, id, pos):
        self.protocol.write(id, AX12MemoryMap.GOAL_POSITION.value, struct.pack('<H', pos))

    def move_speed(self ,id ,pos, speed):
        self.protocol.write(id, AX12MemoryMap.GOAL_POSITION.value, struct.pack('<HH', pos, speed))
    
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

    GOAL_SPEED = 32 # 2 bytes

#  R_ModelNumber             = 0x00, // 2 Byte
# 	    R_FirmwareVersion         = 0x02, //
# 	    R_ServoID                 = 0x03, //         Write
# 	    R_BaudRate                = 0x04, //         Write
# 	    R_ReturnDelayTime         = 0x05, //         Write
# 	    R_CW_AngleLimit           = 0x06, // 2 Byte  Write
# 	    R_CCW_AngleLimit          = 0x08, // 2 Byte  Write
# 	    R_HighestLimitTemperature = 0x0B, //         Write
# 	    R_LowestLimitVoltage      = 0x0C, //         Write
# 	    R_HighestLimitVoltage     = 0x0D, //         Write
# 	    R_MaxTorque               = 0x0E, // 2 Byte  Write
# 	    R_StatusReturnLevel       = 0x10, //         Write
# 	    R_AlarmLED                = 0x11, //         Write
# 	    R_AlarmShutdown           = 0x12, //         Write

# 	    // ----------- RAM -------------

# 	    R_TorqueEnable            = 0x18, //         Write
# 	    R_LED                     = 0x19, //         Write
# 	    R_CW_ComplianceMargin     = 0x1A, //         Write
# 	    R_CCW_ComplianceMargin    = 0x1B, //         Write
# R_CW_ComplianceSlope      = 0x1C, //         Write
# R_CCW_ComplianceSlope     = 0x1D, //         Write
# R_GoalPosition            = 0x1E, // 2 Byte  Write
# R_MovingSpeed             = 0x20, // 2 Byte  Write
# R_TorqueLimit             = 0x22, // 2 Byte  Write
# R_PresentPosition         = 0x24, // 2 Byte
# R_PresentSpeed            = 0x26, // 2 Byte
# R_PresentLoad             = 0x28, // 2 Byte
# R_PresentVoltage          = 0x2A, //
# R_PresentTemperature      = 0x2B, //
# R_RegisteredInstruction   = 0x2C, //         Write
# R_Moving                  = 0x2E, //
# R_Lock                    = 0x2F, //         Write
# R_Punch                   = 0x30, // 2 Byte  Write