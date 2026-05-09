#!/usr/bin/env python3
import threading
import serial
import time
import struct
from enum import Enum
from typing import List, Tuple, Dict, Any
import sys


class Xbee(threading.Thread):
    XBEE_API_START = 0x7E
    GROUND_STATION_ADDR = 0x0100
    DEFAULT_TIMEOUT = 0.1

    class RxState(Enum):
        WAIT_START = 0
        GET_LEN = 1
        GET_FRAME_DATA = 2

    class ATStatus(Enum):
        Ok = 0
        Error = 1
        InvalidCmd = 2
        InvalidParam = 3

    class APITypes(Enum):
        MODEM_STATUS = 0x8A
        AT_CMD = 0x08
        AT_CMD_QU = 0x09
        AT_CMD_RES = 0x88
        REMOTE_AT_REQ = 0x17
        REMOTE_AT_RES = 0x97
        TX_64 = 0x00
        TX_16 = 0x01
        TX_STATUS = 0x89
        RX_64 = 0x80
        RX_16 = 0x81
        RX_IO_64 = 0x82
        RX_IO_16 = 0x83

    @staticmethod
    def from16(nb: int):
        return nb.to_bytes(2, 'big')

    @staticmethod
    def to16(data: bytes):
        return int.from_bytes(data, 'big')

    def __init__(self, callback, id, port, baudrate=57600, verbose=False):
        threading.Thread.__init__(self)
        self.id = int(id)
        self.ser = serial.Serial(port, baudrate)
        self.rx_state = Xbee.RxState.WAIT_START
        self.bytes_needed = 1
        self.running = True
        self._frame_id = 0
        self.responses = {}     # type: Dict[int, Any]
        self.verbose = verbose
        self.callback = callback
        self.buffer = bytearray()

    def start(self) -> None:
        threading.Thread.start(self)
        self.at_cmd(b'MY', self.from16(self.id), resp=False)

    def stop(self):
        if self.running:
            self.running = False
            time.sleep(0.1)
            self.ser.close()

    def __del__(self):
        self.stop()

    @staticmethod
    def checksum(frame):
        return 0xFF - (sum(frame) & 0xFF)

    @property
    def frame_id(self):
        self._frame_id = (self._frame_id + 1) % 0xFF
        return self._frame_id

    def rx_16_cb(self, source, rssi, options, data: bytes):
        if self.callback is not None:
            self.callback(source, data)
        else:
            print(f"No CB; msg from {source}: {data}")

    def handle_rcv(self, frame):
        frame_type = frame[0]
        if frame_type == self.APITypes.MODEM_STATUS.value:
            pass    # TODO
        elif int(frame_type) == self.APITypes.AT_CMD_RES.value:
            frame_id = frame[1]
            at_cmd = frame[2:4]
            status = self.ATStatus(frame[4])
            data = frame[5:]
            self.responses[frame_id] = (status, data)
        elif frame_type == self.APITypes.RX_16.value:
            source_addr = self.to16(frame[1:3])
            rssi = frame[3]
            options = frame[4]
            data = frame[5:]
            self.rx_16_cb(source_addr, rssi, options, data)
        elif frame_type == self.APITypes.TX_STATUS.value:
            frame_id = frame[1]
            status = frame[2]
            self.responses[frame_id] = status
        else:
            print("unknown frame type:", hex(frame_type))

    def send_data(self, dest: int, data, ack=False, options=0):
        if ack:
            frame_id = self.frame_id
        else:
            frame_id = 0
        frame = struct.pack(">BBHB", self.APITypes.TX_16.value, frame_id, dest, options) + data
        self.transmit_frame(frame)

    def get_response_timeout(self, frame_id, timeout=DEFAULT_TIMEOUT) -> Any:
        if frame_id == 0:
            return None
        start_time = time.time()
        while frame_id not in self.responses:
            if time.time() - start_time > timeout:
                raise TimeoutError()
            time.sleep(0.01)
        return self.responses.pop(frame_id)

    def at_cmd(self, at, value=None, resp=True):
        if resp:
            frame_id = self.frame_id
        else:
            frame_id = 0
        cmd = struct.pack("<BB", 0x08, frame_id) + at
        if value is not None:
            cmd += value
        self.transmit_frame(cmd)
        return self.get_response_timeout(frame_id)

    def transmit_frame(self, frame):
        data = struct.pack(">BH", self.XBEE_API_START, len(frame)) + frame + struct.pack("<B", self.checksum(frame))
        self.ser.write(data)
        # self.ser.flush()

    def run(self):
        while self.running:
            self.buffer.extend(self.ser.read())
            if len(self.buffer) < self.bytes_needed:
                continue
            data = self.buffer[0:self.bytes_needed]
            self.buffer = self.buffer[self.bytes_needed:]
            if self.rx_state == self.RxState.WAIT_START:
                if data[0] == self.XBEE_API_START:
                    self.rx_state = self.RxState.GET_LEN
                    self.bytes_needed = 2
            elif self.rx_state == self.RxState.GET_LEN:
                self.bytes_needed = ((data[0] << 8) + data[1]) + 1  # Add 1 byte for the checksum
                self.rx_state = self.RxState.GET_FRAME_DATA
            elif self.rx_state == self.RxState.GET_FRAME_DATA:
                frame = data[:-1]
                chk = data[-1]
                if self.checksum(frame) == chk:
                    self.handle_rcv(frame)
                else:
                    print("invalid chk: {}  {}".format(sum(frame), chk))
                self.rx_state = self.RxState.WAIT_START
                self.bytes_needed = 1
        print("stopped")


if __name__ == '__main__':
    """
    ./xbee.py /dev/ttyUSB0 12 42
    ./xbee.py /dev/ttyUSB0 42 12
    """
    def cb(sender, data):
        print(f"msg from {sender}: {data.decode()}")

    x=Xbee(cb, sys.argv[2], sys.argv[1])
    x.start()
    while True:
        x.send_data(int(sys.argv[3]), b'plop')
        time.sleep(1)

