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


class Rcv:
    def __init__(self,name='') -> None:
        self.img = np.zeros((480, 640, 3))
        ecal_core.initialize([], "test_opencv_receiver")
        self.sub = ProtoSubscriber("images"+str(name), cipb.CompressedImage)
        self.sub.set_callback(self.on_img)

    def on_img(self, topic, msg: cipb.CompressedImage, t):
        # Reconvertit les bytes en tableau numpy
        nparr = np.frombuffer(msg.data, np.uint8)
        # Décode l'image
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        self.img = cv2.rotate(img, cv2.ROTATE_180)




if __name__ == "__main__":
    if len(sys.argv) < 1:
        print("Usage: ./cam_ecal_rcv.py <name>")
        exit(1)

    try:
        cam_name ='_' + str(sys.argv[1])
        rcv = Rcv(cam_name)
    except IndexError:
        rcv=Rcv()

    while True:
        cv2.imshow('Camera', rcv.img)
        # Press 'q' to exit the loop
        if cv2.waitKey(10) == ord('q'):
            break

