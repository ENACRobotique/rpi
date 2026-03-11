#!/usr/bin/env python3
import sys
import time
import ecal.nanobind_core as ecal_core
import struct
from enum import Enum

from sap_instructions import SAPInstructions
from sts3032 import STS3032
from ax12 import AX12
from scs0009 import SCS0009
from pump import Pump


class ActuatorType(Enum):
    STS3032 = 0
    AX12 = 1
    SCS009 = 2
    PUMP = 3
    UNKNOWN = 255


class SAPMaster:
    def __init__(self) -> None:
        self.protocol = SAPInstructions()

        self.sts3032 = STS3032(self.protocol)
        self.scs0009 = SCS0009(self.protocol)
        self.ax12 = AX12(self.protocol)
        self.pump = Pump(self.protocol)

        # associate IDs with actuator type
        self.types_id: dict[int, ActuatorType] = {}
    
    def getActuatorType(self, id):
        # TODO
        return ActuatorType.UNKNOWN
    
    def mapBus(self):
        {id: self.getActuatorType(id) for id in range(254)}
    
    def ping(self, id):
        return self.protocol.ping(id)

if __name__ == "__main__":
    s = SAPMaster()
    ping_results = []
    t_start = time.time()
    for id in range(254):
        ret = s.ping(id)
        t_end = time.time()
        dt = t_end - t_start
        t_start = t_end
        if ret:
            ping_results.append(id)
            print(f"{dt:.2f} ID: {id} SUCCESS!")
        else:
            print(f"{dt:.2f} ID: {id} not responding")
    print(ping_results)
    
    
        
    



