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
import argparse
import time


if __name__ == "__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument('cam', type=int, help='Camera ID', default=None)
    parser.add_argument('-t', '--topic', help='eCAL topic', default="cam")
    parser.add_argument('-W', '--width', type=int, help='image width', default=None)
    parser.add_argument('-H', '--height', type=int, help='image height', default=None)
    parser.add_argument('-f', '--fps', type=int, help='framerate', default=None)
    args = parser.parse_args()

    if not ecal_core.is_initialized():
        ecal_core.initialize("cam2ecal")

    pub = ProtoPublisher(cipb.CompressedImage, args.topic)
    cam = cv2.VideoCapture(args.cam)
    if not cam.isOpened():
        print("Can't open camera!!!")
        exit(-1)
    else:
        print(f"cam {args.cam} opened.")
    
    if args.width is not None and args.height is not None:
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if args.fps is not None:
        print(f"setting fps at {args.fps}...")
        cam.set(cv2.CAP_PROP_FPS, args.fps)

    # Get the frame width and height
    w, h, f = cam.get(cv2.CAP_PROP_FRAME_WIDTH), cam.get(cv2.CAP_PROP_FRAME_HEIGHT), cam.get(cv2.CAP_PROP_FPS)
    print(f"Opened camera with resolution {w}x{h} at {f}fps!\n")

    while True:
        ret, frame = cam.read()
        img_encode = cv2.imencode(".jpg", frame)[1]
        byte_encode = img_encode.tobytes()
        ci = cipb.CompressedImage(timestamp=Timestamp(), data=byte_encode, format='jpeg')
        pub.send(ci)
