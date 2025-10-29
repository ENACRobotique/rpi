#!/usr/bin/env python3
import cv2
import numpy as np
import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '../..')) # Avoids ModuleNotFoundError when finding generated folder
import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber
from ecal.core.publisher import ProtoPublisher
from generated.robot_state_pb2 import Position_aruco
from generated import CompressedImage_pb2 as cipb
from google.protobuf.timestamp_pb2 import Timestamp
import argparse
from enum import Enum
from scipy.spatial.transform import Rotation
import time


class Source(Enum):
    CAM = 0
    VIDEO = 1
    ECAL = 2


class ArucoFinder:
    def __init__(self, name, src_type, src, arucos, display):
        if not ecal_core.is_initialized():
            ecal_core.initialize(sys.argv, "arucoFinder")
        
        self.name = name
        self.src_type = src_type
        self.src = src
        self.arucos = arucos
        self.display = display
        
        if src_type == Source.CAM:
            self.cap = cv2.VideoCapture(src)
            if self.cap is None:
                print(f"Failed to open {self.name} with id {src}")
                return
            #self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
            #self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
            w, h = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH), self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            print(f"Opened camera with resolution {w}x{h}!\n")
        elif src_type == Source.VIDEO:
            self.cap = cv2.VideoCapture(src)
            if self.cap is None:
                print(f"Failed to open {self.name} with id {src}")
                return
            w, h = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH), self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            print(f"Opened video with resolution {w}x{h}!\n")
        elif src_type == Source.ECAL:
            self.sub = ProtoSubscriber(src, cipb.CompressedImage)
            self.sub.set_callback(self.on_img)
        
        if self.display:
            self.cam_pub = ProtoPublisher("images_"+str(self.name), cipb.CompressedImage)
        
        
        
        self.aruco_pub = ProtoPublisher("Arucos", Position_aruco)
        

        # ArUco settings (API OpenCV 4.7+)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        self.resolution = (640, 480) #width, height
        self.getCalibration(f'{self.name}_matrix_1920x1080.npy', f'{self.name}_coeffs_1920x1080.npy', (1920,1080))

    def getCalibration(self, matrix, coefs, resolution = (640,480)):
        """Provide Calibration Matrix and distance coefs as .npy file"""
        # Charger la calibration
        self.camera_matrix = np.load(matrix)
        self.dist_coeffs = np.load(coefs)
        self.resolution = resolution
    
    def on_img(self, topic, msg: cipb.CompressedImage, t):
        nparr = np.frombuffer(msg.data, np.uint8)
        # Décode l'image
        frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        processed = self.process(frame)
        if self.display:
            self.send_processed_frame(processed)

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
            xs, ys, zs, aruIds= [],[],[],[]
            qws, qxs, qys, qzs = [],[],[],[]
            for corners, id in zip(detected_corners, detected_ids):
                id = id[0]
                if id not in self.arucos:
                    continue
                size = self.arucos[id]
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers([corners], size, self.camera_matrix, self.dist_coeffs)
                if tvecs is not None:
                    rv, tv = rvecs[0], tvecs[0]
                    xs.append(tv[0][2])
                    ys.append(tv[0][0])
                    zs.append(-tv[0][1])
                    aruIds.append(id)

                    # Convert rvec to rotation matrix
                    rotation_matrix, _ = cv2.Rodrigues(np.array(rv))
                    ### first transform the matrix to euler angles
                    r =  Rotation.from_matrix(rotation_matrix)
                    (qx, qy, qz, qw) = r.as_quat(False)
                    qxs.append(qx)
                    qys.append(qy)
                    qzs.append(qz)
                    qws.append(qw)
            self.arucoFound = Position_aruco(x=xs, y=ys, z=zs, qx=qxs, qy=qys, qz=qzs, qw=qws, ArucoId=aruIds, cameraName=self.name)
            self.aruco_pub.send(self.arucoFound)
        
        return frame
    
    def run(self):
        while True:
            if self.src_type == Source.CAM or self.src_type == Source.VIDEO:
                ret, frame = self.cap.read()
                processed = self.process(frame)
                if self.display:
                    self.send_processed_frame(processed)
            else:
                time.sleep(1)


if __name__ == "__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument('name', help='camera name')
    parser.add_argument('-c', '--cam', type=int, help='Camera ID', default=None)
    parser.add_argument('-v', '--video', help='Video file', default=None)
    parser.add_argument('-t', '--topic', help='eCAL topic', default=None)
    parser.add_argument('-d', '--display', action='store_true', default=False, help='send annotetd images over ecal')
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
    else:
        print("Please specify the source: cam, video or ecal topic.")
    
    arucos = {47:30, 36:30, 20:100, 21:100, 22:100, 23:100}

    af = ArucoFinder(args.name, src_type, src, arucos, args.display)

    while ecal_core.ok():
            af.run()

