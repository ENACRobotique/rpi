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

plotjuggler_udp = ("192.168.42.119", 9870)


class RxState(Enum):
    IDLE = 0
    HEAD_OK = 1
    GOT_LENGTH = 2
    

class Duckoder(Protocol):
    def __init__(self, plotjuggler=False):
        Protocol.__init__(self)
        self.transport = None
        self._buffer = b'  '
        self._rx_state = RxState.IDLE
        self._msg_rcv = None
        self.send_plotjuggler = plotjuggler
        if self.send_plotjuggler:
            self.so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        ecal_core.initialize(sys.argv, "Bridge low level")
        self.odom_pos_pub = ProtoPublisher("odom_pos", hgpb.Position)
        self.odom_move_pub = ProtoPublisher("odom_move", hgpb.Position)
        self.odom_speed_pub = ProtoPublisher("odom_speed", hgpb.Speed)
        self.carrot_pos_pub = ProtoPublisher("carrot_pos", hgpb.Position)
        self.ins_pub = ProtoPublisher("ins", hgpb.Ins)
        self.target_pos_sub = ProtoSubscriber("set_position", hgpb.Position)
        self.reset_pos_sub = ProtoSubscriber("reset", hgpb.Position)
        self.pid_sub = ProtoSubscriber("pid_gains",llpb.MotorPid)
        self.speed_cons_sub = ProtoSubscriber("speed_cons",hgpb.Speed)
        
        self.target_pos_sub.set_callback(self.set_target)
        self.reset_pos_sub.set_callback(self.reset_position)
        self.pid_sub.set_callback(self.set_pid)
        self.speed_cons_sub.set_callback(self.set_speed)
        

    def connection_made(self, transport):
        self.transport = transport

    def data_received(self, data):
        for c in data:
            if self._decode(c.to_bytes(1, 'little')):
                m = llpb.Message.FromString(self._msg_rcv)
                if self.send_plotjuggler:
                    jj = self.msg_to_json(m)
                    #print(jj)
                    self.so.sendto(jj.encode(), plotjuggler_udp)
                topic = m.WhichOneof('inner')
                if topic == "pos" and m.msg_type == llpb.Message.MsgType.STATUS:
                    hgm = hgpb.Position(x=m.pos.x,y=m.pos.y,theta=m.pos.theta)
                    if m.pos.obj == llpb.Pos.PosObject.POS_ROBOT_W:
                        self.odom_pos_pub.send(hgm)
                    elif m.pos.obj == llpb.Pos.PosObject.POS_CARROT_W:
                        self.carrot_pos_pub.send(hgm)
                    elif m.pos.obj == llpb.Pos.PosObject.MOVE_ROBOT_R:
                        self.odom_move_pub.send(hgm)
                if topic == "speed" and m.msg_type == llpb.Message.MsgType.STATUS:
                    hgm = hgpb.Speed(vx=m.speed.vx,vy=m.speed.vy,vtheta=m.speed.vtheta)
                    self.odom_speed_pub.send(hgm)
                if topic == "ins" and m.msg_type == llpb.Message.MsgType.STATUS:
                    hgm = hgpb.Ins(vtheta=m.ins.vtheta,theta=m.ins.theta)
                    self.ins_pub.send(hgm)
                

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

    def msg_to_json(self, msg):
        msg_name = msg.WhichOneof('inner')
        inner = getattr(msg, msg_name)
        if msg_name == 'motors':
            msg_name = msg.motors.MotorDataType.Name(msg.motors.type)
        elif msg_name == 'pos':
            msg_name = msg.pos.PosObject.Name(msg.pos.obj)
        d = {msg_name: {}}
        for f in inner.DESCRIPTOR.fields:
            field_name = f.name
            d[msg_name][field_name] = getattr(inner, field_name)
        return json.dumps(d)


    def send_message(self, msg: llpb.Message):
        payload = msg.SerializeToString()
        lenght = len(payload)
        crc = 0
        for b in payload:
            crc ^= b
        buffer = struct.pack('<BBB', 0xFF, 0xFF, lenght) + payload + struct.pack('<B', crc)
        # print([c for c in buffer])
        self.transport.write(buffer)
    
    def set_target(self, topic_name, hlm, time):
        llmsg = llpb.Message()
        llmsg.msg_type = llpb.Message.MsgType.COMMAND
        llmsg.pos.x = hlm.x
        llmsg.pos.y = hlm.y
        llmsg.pos.theta = hlm.theta
        llmsg.pos.obj = llpb.Pos.PosObject.POS_ROBOT_W
        self.send_message(llmsg)

    def reset_position(self, topic_name, hlm, time):
        llmsg = llpb.Message()
        llmsg.msg_type = llpb.Message.MsgType.COMMAND
        llmsg.pos.x = hlm.x
        llmsg.pos.y = hlm.y
        llmsg.pos.theta = hlm.theta
        llmsg.pos.obj = llpb.Pos.PosObject.RECALAGE
        self.send_message(llmsg)

    def set_pid(self, topic_name, hlm, time):
        llmsg = llpb.Message()
        llmsg.msg_type = llpb.Message.MsgType.COMMAND
        llmsg.motor_pid.motor_no = hlm.motor_no
        llmsg.motor_pid.kp = hlm.kp
        llmsg.motor_pid.ki = hlm.ki
        llmsg.motor_pid.kd = hlm.kd
        self.send_message(llmsg)

    def set_speed(self, topic_name, hlm, time):
        llmsg = llpb.Message()
        llmsg.msg_type = llpb.Message.MsgType.COMMAND
        llmsg.speed.vx = hlm.vx
        llmsg.speed.vy = hlm.vy
        llmsg.speed.vtheta = hlm.vtheta
        self.send_message(llmsg)
        


if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/robot_base"

    ser=Serial(port, 115200)
    with ReaderThread(ser, Duckoder) as p:
        while True:
            time.sleep(1)

