#!/usr/bin/env python3
import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber
from ecal.core.publisher import ProtoPublisher
import sys
import cv2
import numpy as np
from generated import CompressedImage_pb2 as cipb
from google.protobuf.timestamp_pb2 import Timestamp
import time

ecal_core.initialize([], "test_opencv")

pub = ProtoPublisher("images", cipb.CompressedImage)

cam = cv2.VideoCapture(8)
if not cam.isOpened():
    print("Can't open camera!!!")
    exit(-1)
else:
    print("cam opened")

# Get the default frame width and height
frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

fid = 0

while True:
    ret, frame = cam.read()
    # Display the captured frame
    #cv2.imshow('Camera', frame)

    img_encode = cv2.imencode(".jpg", frame)[1]
    byte_encode = img_encode.tobytes()
    #timestamp = Timestamp()
    ci = cipb.CompressedImage(timestamp=Timestamp(), data=byte_encode, format='jpeg')
    pub.send(ci)

    fid += 1


