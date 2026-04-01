#!/usr/bin/env python3
import cv2
import numpy as np
import os, sys
import argparse
from enum import Enum
import ecal.nanobind_core as ecal_core
from ecal.msg.proto.core import Subscriber as ProtoSubscriber
from ecal.msg.common.core import ReceiveCallbackData
from generated import CompressedImage_pb2 as cipb
from threading import Event

"""
A good camera calibration needs:
    - fixed focus
    - the checkboard pattern should cover at least 20% of the image
    - the pattern should be at an angle less than 45° from the camera plane
    - capture the pattern at different orientations, and all over the image
"""

# Paramètres du damier
NB_CORNERS_X = 9  # coins internes horizontalement
NB_CORNERS_Y = 6  # coins internes verticalement
SQUARE_SIZE_MM = 17.7  # taille réelle d’un carré en mm


class Source(Enum):
    CAM = 0
    VIDEO = 1
    ECAL = 2

class Calibrator:
    def __init__(self, name, src_type, src, args) -> None:
        self.name = name
        self.src_type = src_type
        self.dir = args.dir

        self.init_capture(src_type, src, args)

        # Préparation des points 3D de référence (repère monde)
        self.objp = np.zeros((NB_CORNERS_Y * NB_CORNERS_X, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:NB_CORNERS_X, 0:NB_CORNERS_Y].T.reshape(-1, 2)
        self.objp *= SQUARE_SIZE_MM / 1000.0  # en mètres

        # Listes de points pour calibration
        self.objpoints = []  # points 3D
        self.imgpoints = []  # points 2D

        self.imageSize = (0, 0)
        self.img: np.ndarray = None
        self.event = Event()


    def init_capture(self, src_type, src, args):
        if src_type == Source.CAM:
            self.cap = cv2.VideoCapture(src)
            if self.cap is None:
                print(f"Failed to open camera {src}")
                return
            if args.fourcc is not None:
                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*args.fourcc))
            if args.width is not None and args.height is not None:
                print(f"setting resoltution at {args.width}x{args.height}....")
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
                print(f"Failed to open file {src}")
                return
            w, h = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH), self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            print(f"Opened video with resolution {w}x{h}!\n")
        elif src_type == Source.ECAL:
            if not ecal_core.is_initialized():
                ecal_core.initialize("arucoFinder")
            self.sub = ProtoSubscriber(cipb.CompressedImage, src)
            self.sub.set_receive_callback(self.on_img)
    
    def stop_capture(self):
        if src_type == Source.CAM:
            self.cap.release()
        elif src_type == Source.VIDEO:
            self.cap.release()
        elif src_type == Source.ECAL:
            pass
    
    def on_img(self, pub_id: ecal_core.TopicId, data: ReceiveCallbackData[cipb.CompressedImage]):
        nparr = np.frombuffer(data.message.data, np.uint8)
        frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        self.img = frame
        self.event.set()

    
    def run(self):
        print("Appuie sur 'espace' pour capturer une image, 'q' pour quitter.")

        while True:
            if src_type == Source.CAM or src_type == Source.VIDEO:
                ret, frame = self.cap.read()
                if not ret:
                    break
            elif src_type == Source.ECAL:
                self.event.wait()
                frame = self.img.copy()

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ret_corners, corners = cv2.findChessboardCorners(gray, (NB_CORNERS_X, NB_CORNERS_Y), None)
            if ret_corners:
                cv2.drawChessboardCorners(frame, (NB_CORNERS_X, NB_CORNERS_Y), corners, ret_corners)

            cv2.imshow('Calibration', frame)
            
            key = cv2.waitKey(1)

            if key == ord(' '):
                if ret_corners:
                    self.objpoints.append(self.objp)
                    self.imgpoints.append(corners)
                    self.imageSize = gray.shape[::-1]
                    print(self.imageSize)
                    print(f"[{len(self.objpoints)}] Capture enregistrée.")
                else:
                    print("❌ Damier non détecté.")

            elif key == ord('q'):
                break

        self.stop_capture()
        cv2.destroyAllWindows()
        self.process()

    def process(self):
        if len(self.objpoints) >= 10:
            print("📐 Calibration en cours...")
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(self.objpoints, self.imgpoints, self.imageSize, None, None)
            w, h = self.imageSize
            filename_calib = f'{self.dir}/{self.name}_{w}x{h}.yml'
            fs = cv2.FileStorage(filename_calib, cv2.FILE_STORAGE_WRITE)
            fs.write("camera_matrix", mtx)
            fs.write("dist_coeffs", dist)
            fs.release()

            print("✅ Calibration réussie ! Fichier enregistré :")
            print(f"- {filename_calib}")
        else:
            print("⚠️ Pas assez d'images pour calibrer (au moins 10 bonnes prises nécessaires)")


if __name__ == "__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument('name', help='Camera name')
    parser.add_argument('-c', '--cam', type=int, help='Camera ID', default=None)
    parser.add_argument('-v', '--video', help='Video file', default=None)
    parser.add_argument('-t', '--topic', help='eCAL topic', default=None)
    parser.add_argument('-W', '--width', type=int, help='image width, for camera', default=None)
    parser.add_argument('-H', '--height', type=int, help='image height, for camera', default=None)
    parser.add_argument('-f', '--fps', type=int, help='framerate', default=None)
    parser.add_argument('--fourcc', type=str, help='fourcc type (MJPG, H264, ...)', default=None)
    parser.add_argument('-d', '--dir', default='../../data/camera_calibrations/', help='Directory for calibration files')
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
    
    calibrator = Calibrator(args.name, src_type, src, args)
    calibrator.run()
