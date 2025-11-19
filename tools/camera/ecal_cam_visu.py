#!/usr/bin/env python3
import ecal.nanobind_core as ecal_core
from ecal.msg.proto.core import Subscriber as ProtoSubscriber
from ecal.msg.proto.core import Publisher as ProtoPublisher
from ecal.msg.common.core import ReceiveCallbackData
import sys
import cv2
import numpy as np
from generated import CompressedImage_pb2 as cipb
from google.protobuf.timestamp_pb2 import Timestamp
import time
import argparse
from threading import Event
from enum import Enum


class CamVisu:
    def __init__(self, args) -> None:
        if not ecal_core.is_initialized():
            ecal_core.initialize("Ecal cam visu")
        self.sub = ProtoSubscriber(cipb.CompressedImage, args.topic)
        self.img = None
        self.rotate = args.rotate
        self.event = Event()
        self.sub.set_receive_callback(self.on_img)

    def on_img(self, pub_id: ecal_core.TopicId, data: ReceiveCallbackData[cipb.CompressedImage]):
        nparr = np.frombuffer(data.message.data, np.uint8)
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        match self.rotate:
            case '0':
                self.img = img
            case '180':
                self.img = cv2.rotate(img, cv2.ROTATE_180)
                self.event.set()
            case '90':
                self.img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
                self.event.set()
            case '90ccw':
                self.img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
        self.event.set()
        
    
    def run(self):
        while True:
            self.event.wait()
            cv2.imshow('Camera', self.img)
            # Press 'q' to exit the loop
            if cv2.waitKey(1) == ord('q'):
                break


if __name__ == "__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument('-t', '--topic', help='eCAL topic', default="cam")
    parser.add_argument('-r', '--rotate', choices=['180', '90', '90ccw'], default='0')
    args = parser.parse_args()

    rcv = CamVisu(args)
    rcv.run()
