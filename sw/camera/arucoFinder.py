#!/usr/bin/env python3
import cv2
import numpy as np
import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '../..')) # Avoids ModuleNotFoundError when finding generated folder
import ecal.nanobind_core as ecal_core
from ecal.msg.proto.core import Subscriber as ProtoSubscriber
from ecal.msg.proto.core import Publisher as ProtoPublisher
from ecal.msg.common.core import ReceiveCallbackData
from generated.robot_state_pb2 import Aruco, Arucos
from generated import CompressedImage_pb2 as cipb
from google.protobuf.timestamp_pb2 import Timestamp
import argparse
from enum import Enum
from scipy.spatial.transform import Rotation
import time
from threading import Event

GST_PIPELINE = (
    'udpsrc port=5000 caps="application/x-rtp,media=video,encoding-name=H265,payload=96,clock-rate=90000" ! '
    'rtph265depay ! avdec_h265 ! videoconvert ! '
    'video/x-raw,format=BGR ! '
    'appsink drop=1 sync=false max-buffers=1'
)

class Source(Enum):
    CAM = 0
    VIDEO = 1
    ECAL = 2
    GSTREAMER = 3


class ArucoFinder:
    def __init__(self, name, src_type, src, arucos, display):
        if not ecal_core.is_initialized():
            ecal_core.initialize("arucoFinder")
        
        self.name = name
        self.src_type = src_type
        self.src = src
        self.arucos = arucos
        self.display = display
        
        self.event = Event()
        self.img = None     # img received from eCAL

        self.aruco_pub = ProtoPublisher(Arucos, "Arucos")
        if self.display:
            self.cam_pub = ProtoPublisher(cipb.CompressedImage, "images_"+str(self.name))
        
        # ArUco settings (API OpenCV 4.7+)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.camera_matrix = None
        self.dist_coeffs = None
        
        self.open_capture()

    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_value, traceback):
        if src_type == Source.ECAL:
            self.sub.remove_receive_callback()
    
    def open_capture(self):
        if src_type == Source.CAM:
            self.cap = cv2.VideoCapture(src)
            if self.cap is None:
                print(f"Failed to open {self.name} with id {src}")
                exit(-1)
            if args.fourcc is not None:
                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*args.fourcc))
            if args.width is not None and args.height is not None:
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
            if args.fps is not None:
                print(f"setting fps at {args.fps}...")
                self.cap.set(cv2.CAP_PROP_FPS, args.fps)
            w, h, f = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH), self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT), self.cap.get(cv2.CAP_PROP_FPS)
            print(f"Opened camera with resolution {w}x{h} at {f}fps!\n")
        elif src_type == Source.VIDEO:
            self.cap = cv2.VideoCapture(src)
            if self.cap is None:
                print(f"Failed to open {self.name} with id {src}")
                return
            w, h = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH), self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            print(f"Opened video with resolution {w}x{h}!\n")
        elif src_type == Source.ECAL:
            self.sub = ProtoSubscriber(cipb.CompressedImage, src)
            self.sub.set_receive_callback(self.on_img)
        elif src_type == Source.GSTREAMER:
            print(f"Open GST pipeline: {src}")
            self.cap = cv2.VideoCapture(src, cv2.CAP_GSTREAMER)
            if self.cap.isOpened():
                print("Gstreamer pipeline opened.")
            else:
                print("GStreamer failed to open.")

    def getCalibration(self, w, h):
        """Provide Calibration Matrix and distance coefs as .npy file"""
        # Charger la calibration
        f_calib = f'../../data/camera_calibrations/{self.name}_{w}x{h}.yml'
        fs = cv2.FileStorage(f_calib, cv2.FILE_STORAGE_READ)
        self.camera_matrix = fs.getNode("camera_matrix").mat()
        self.dist_coeffs = fs.getNode("dist_coeffs").mat()
        fs.release()

    
    def on_img(self, pub_id: ecal_core.TopicId, data: ReceiveCallbackData[cipb.CompressedImage]):
        nparr = np.frombuffer(data.message.data, np.uint8)
        self.img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        self.event.set()

    def send_processed_frame(self, frame):
        img_encode = cv2.imencode(".jpg", frame)[1]
        byte_encode = img_encode.tobytes()
        ci = cipb.CompressedImage(timestamp=Timestamp(), data=byte_encode, format='jpeg')
        self.cam_pub.send(ci)

    def process(self, frame):
        """Call it in a while true loop"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Détection ArUco
        detected_corners, detected_ids, rejected = self.aruco_detector.detectMarkers(gray)

        if self.display:
            cv2.aruco.drawDetectedMarkers(frame, detected_corners, detected_ids)

        if detected_corners:
            arucos = []
            for corners, id in zip(detected_corners, detected_ids):
                id = id[0]
                if id not in self.arucos:
                    continue
                size = self.arucos[id]
                
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers([corners], size, self.camera_matrix, self.dist_coeffs)
                if tvecs is not None:
                    rv, tv = rvecs[0], tvecs[0]

                    # Convert rvec to rotation matrix
                    rotation_matrix, _ = cv2.Rodrigues(np.array(rv))
                    r =  Rotation.from_matrix(rotation_matrix)
                    (qx, qy, qz, qw) = r.as_quat(False)


                    ar = Aruco(x=tv[0][0], y=tv[0][1], z=tv[0][2],qx=qx, qy=qy, qz=qz, qw=qw, ArucoId=id )
                    arucos.append(ar)


            self.arucoFound = Arucos(arucos=arucos, cameraName=self.name)
            self.aruco_pub.send(self.arucoFound)
        return frame
    
    def run(self):
        while True:
            if self.src_type == Source.CAM or self.src_type == Source.VIDEO or self.src_type == Source.GSTREAMER:
                ret, frame = self.cap.read()
            else:
                self.event.wait()
                frame = self.img.copy()
            
            if self.camera_matrix is None:
                h, w, _ = frame.shape
                self.getCalibration(w, h)
            
            processed = self.process(frame)
            if self.display:
                self.send_processed_frame(processed)


if __name__ == "__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument('name', help='camera name')
    parser.add_argument('-c', '--cam', type=str, help='Camera path (/dev/video)', default='/dev/video0')
    parser.add_argument('-v', '--video', help='Video file', default=None)
    parser.add_argument('-g', '--gstreamer', action='store_true', help='Video file', default=None)
    parser.add_argument('-t', '--topic', help='eCAL topic', default=None)
    parser.add_argument('-d', '--display', action='store_true', default=False, help='send annotated images over ecal')
    parser.add_argument('-W', '--width', type=int, help='image width', default=None)
    parser.add_argument('-H', '--height', type=int, help='image height', default=None)
    parser.add_argument('-f', '--fps', type=int, help='framerate', default=None)
    parser.add_argument('--fourcc', type=str, help='fourcc type (MJPG, H264, ...)', default=None)
    args = parser.parse_args()

    if args.cam is not None:
        src_type = Source.CAM
        src = args.cam
    elif args.video is not None:
        src_type = Source.VIDEO
        src = args.video
    elif args.topic is not None:
        src_type = Source.ECAL
        src = args.topic
    elif args.gstreamer is not None:
        src_type = Source.GSTREAMER
        src = GST_PIPELINE
    else:
        print("Please specify the source: cam, video or ecal topic.")
    
    arucos = {47:30, 36:30, 20:100, 21:100, 22:100, 23:100, 13:30}


    with ArucoFinder(args.name, src_type, src, arucos, args.display) as af:
        while ecal_core.ok():
            af.run()#k

