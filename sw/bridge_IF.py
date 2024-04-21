#!/bin/env python3
import sys
sys.path.append('../generated')
from serial import Serial
from serial.threaded import Protocol, ReaderThread
import time
from enum import Enum
import messages_pb2 as llpb
import robot_state_pb2 as hgpb
import json
import socket
import struct
import math

import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher
from ecal.core.subscriber import ProtoSubscriber



#### Est ce que c'est toujours utile ? Ce fichier


class RxState(Enum):
    IDLE = 0
    HEAD_OK = 1
    GOT_LENGTH = 2
    

class IF(Protocol):
    def __init__(self):
        Protocol.__init__(self)
        self.transport = None
        self._buffer = b'  '
        self._rx_state = RxState.IDLE
        self._msg_rcv = None
        ecal_core.initialize(sys.argv, "Bridge Interface")

        self.action_if_pub = ProtoPublisher("action", hgpb.Action)
        self.team_pub = ProtoPublisher("team", hgpb.Team)

        self.color_sub = ProtoSubscriber("color", hgpb.Color)
        self.color_sub.set_callback(self.set_rgb)


    def connection_made(self, transport):
        self.transport = transport


    def data_received(self, data):
        for c in data:
            if self._decode(c.to_bytes(1, 'little')):
                m = llpb.Message.FromString(self._msg_rcv)

                topic = m.WhichOneof('inner')
                if topic == "action":
                    ### On considère un msg type action avec une action OK et une action RETURN et une action TURN
                    hgm = hgpb.Action(ok=m.ok, ret=m.ret, turn = m.turn)
                    self.action_if_pub.send(hgm) 

                elif topic == "team":
                    self.team_pub.send(hgm)

                    
                

    def _decode(self, c):
        ret = False
        self._buffer += c
        match self._rx_state:
            case RxState.IDLE:
                if self._buffer[-1] == 0xFF and self._buffer[-2] == 0xFF:
                    self._buffer = self._buffer[-2:]
                    self._rx_state = RxState.HEAD_OK
            case RxState.HEAD_OK:
                self._expected_bytes = ord(c)+1
                self._rx_state = RxState.GOT_LENGTH
            case  RxState.GOT_LENGTH:
                self._expected_bytes -= 1
                if self._expected_bytes == 0:
                    chk = 0
                    for c in self._buffer[3:-1]:
                        chk ^= c
                    if chk == self._buffer[-1]:
                        ret = True
                        self._msg_rcv = self._buffer[3:-1]
                    else:
                        print("chk failed")
                    self._buffer = b'  '
                    self._rx_state = RxState.IDLE
        return ret


    def send_message(self, msg: llpb.Message):
        payload = msg.SerializeToString()
        lenght = len(payload)
        crc = 0
        for b in payload:
            crc ^= b
        buffer = struct.pack('<BBB', 0xFF, 0xFF, lenght) + payload + struct.pack('<B', crc)
        # print([c for c in buffer])
        self.transport.write(buffer)
    
        

if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/robot_lcd"

    ser=Serial(port, 115200)
    with ReaderThread(ser, IF) as p:
        while True:
            time.sleep(1)

