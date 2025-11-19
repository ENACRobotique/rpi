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
import argparse

import ecal.nanobind_core as ecal_core
from ecal.msg.proto.core import Publisher as ProtoPublisher
from ecal.msg.proto.core import Subscriber as ProtoSubscriber
from ecal.msg.common.core import ReceiveCallbackData


class RxState(Enum):
    IDLE = 0
    HEAD_OK = 1
    GOT_LENGTH = 2
    

class Duckoder(Protocol):
    def __init__(self):
        Protocol.__init__(self)
        self.transport = None
        self._buffer = b'  '
        self._rx_state = RxState.IDLE
        self._msg_rcv = None
        if args.plotjuggler:
            self.so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if not ecal_core.is_initialized():
            ecal_core.initialize("Bridge low level")
        self.odom_pos_pub = ProtoPublisher(hgpb.Position, "odom_pos")
        self.odom_move_pub = ProtoPublisher(hgpb.Position, "odom_move")
        self.odom_speed_pub = ProtoPublisher(hgpb.Speed, "odom_speed")
        self.carrot_pos_pub = ProtoPublisher(hgpb.Position, "carrot_pos")
        self.ins_pub = ProtoPublisher(hgpb.Ins, "ins")
        self.motors_cmd_pub = ProtoPublisher(llpb.Motors, "motors_cmd")
        self.motors_speed_pub = ProtoPublisher(llpb.Motors, "motors_speed")
        self.motors_pos_pub = ProtoPublisher(llpb.Motors, "motors_pos")
        self.motors_pos_cons_pub = ProtoPublisher(llpb.Motors, "motors_pos_cons")

        self.topic_pubs = {
            llpb.Topic.POS_ROBOT_W:     self.odom_pos_pub,
            llpb.Topic.POS_CARROT_W:    self.carrot_pos_pub,
            llpb.Topic.MOVE_ROBOT_R:    self.odom_move_pub
        }

        self.motors_pubs = {
            llpb.Motors.MotorDataType.MOTORS_POS_CONS:  self.motors_pos_cons_pub,
            llpb.Motors.MotorDataType.MOTORS_SPEED:     self.motors_speed_pub,
            llpb.Motors.MotorDataType.MOTORS_CMD:       self.motors_cmd_pub,
            llpb.Motors.MotorDataType.MOTORS_POS:       self.motors_pos_pub,
        }

        self.target_pos_sub = ProtoSubscriber(hgpb.Position, "set_position")
        self.reset_pos_sub = ProtoSubscriber(hgpb.Position, "reset")
        self.pid_sub = ProtoSubscriber(llpb.MotorPid, "pid_gains")
        self.speed_cons_sub = ProtoSubscriber(hgpb.Speed, "speed_cons")

        self.target_pos_sub.set_receive_callback(self.set_target)
        self.reset_pos_sub.set_receive_callback(self.reset_position)
        self.pid_sub.set_receive_callback(self.set_pid)
        self.speed_cons_sub.set_receive_callback(self.set_speed)
        

    def connection_made(self, transport):
        self.transport = transport

    def data_received(self, data):
        for c in data:
            if self._decode(c.to_bytes(1, 'little')):
                m = llpb.Message.FromString(self._msg_rcv)
                if args.plotjuggler:
                    jj = self.msg_to_json(m)
                    #print(jj)
                    self.so.sendto(jj.encode(), (args.addr, args.port))
                inner = m.WhichOneof('inner')
                if m.msg_type == llpb.Message.MsgType.STATUS:
                    if inner == "pos":
                        if m.topic in self.topic_pubs:
                            self.topic_pubs[m.topic].send(m.pos)
                    if inner == "speed":
                        self.odom_speed_pub.send(m.speed)
                    if inner == "ins":
                        self.ins_pub.send(m.ins)
                    if inner == "motors":
                        if m.motors.type in self.motors_pubs:
                            self.motors_pubs[m.motors.type].send(m.motors)


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
            msg_name = llpb.Topic.Name(msg.topic)
        d = {msg_name: {}}
        for f in inner.DESCRIPTOR.fields:
            field_name = f.name
            if f.label == f.LABEL_REPEATED:
                pass
                d[msg_name][field_name] = [di for di in getattr(inner, field_name)]
            else:
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
    
    def set_target(self, pub_id: ecal_core.TopicId, data: ReceiveCallbackData[hgpb.Position]):
        hlm = data.message
        llmsg = llpb.Message(pos=hlm)
        llmsg.msg_type = llpb.Message.MsgType.COMMAND
        llmsg.topic = llpb.Topic.POS_ROBOT_W
        self.send_message(llmsg)

    def reset_position(self, pub_id: ecal_core.TopicId, data: ReceiveCallbackData[hgpb.Position]):
        hlm = data.message
        llmsg = llpb.Message(pos=hlm)
        llmsg.msg_type = llpb.Message.MsgType.COMMAND
        llmsg.topic = llpb.Topic.RECALAGE
        self.send_message(llmsg)

    def set_pid(self, pub_id: ecal_core.TopicId, data: ReceiveCallbackData[llpb.MotorPid]):
        hlm = data.message
        llmsg = llpb.Message()
        llmsg.msg_type = llpb.Message.MsgType.COMMAND
        llmsg.motor_pid.motor_no = hlm.motor_no
        llmsg.motor_pid.kp = hlm.kp
        llmsg.motor_pid.ki = hlm.ki
        llmsg.motor_pid.kd = hlm.kd
        self.send_message(llmsg)

    def set_speed(self, pub_id: ecal_core.TopicId, data: ReceiveCallbackData[hgpb.Speed]):
        hlm = data.message
        llmsg = llpb.Message()
        llmsg.msg_type = llpb.Message.MsgType.COMMAND
        llmsg.speed.vx = hlm.vx
        llmsg.speed.vy = hlm.vy
        llmsg.speed.vtheta = hlm.vtheta
        self.send_message(llmsg)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--serial", default="/dev/robot_base", help="base serial port")
    parser.add_argument("-p", "--plotjuggler", action="store_true", default=False, help="Send data to plotjuggler")
    parser.add_argument("-a", "--addr", default="192.168.42.201", help="Plotjuggler server ip address")
    parser.add_argument("-u", "--port", default=9870, help="Plotjuggler server port", type=int)
    args = parser.parse_args()


    ser=Serial(args.serial, 230400)
    with ReaderThread(ser, Duckoder) as p:
        while True:
            time.sleep(1)

