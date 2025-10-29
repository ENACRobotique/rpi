#!/usr/bin/env python3
import cv2
import numpy as np
import os, sys
import argparse
from enum import Enum
import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber
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
    def __init__(self, name, src_type, src) -> None:
        self.name = name
        self.src_type = src_type
        self.dir = args.dir

        self.init_capture(src_type, src)

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


    def init_capture(self, src_type, src):
        if src_type == Source.CAM:
            self.cap = cv2.VideoCapture(src)
            if self.cap is None:
                print(f"Failed to open camera {src}")
                return
            #self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            #self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
            #self.cap.set(cv2.CAP_PROP_FPS, 30)
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
                ecal_core.initialize(sys.argv, "arucoFinder")
            self.sub = ProtoSubscriber(src, cipb.CompressedImage)
            self.sub.set_callback(self.on_img)
    
    def stop_capture(self):
        if src_type == Source.CAM:
            self.cap.release()
        elif src_type == Source.VIDEO:
            self.cap.release()
        elif src_type == Source.ECAL:
            pass
    
    def on_img(self, topic, msg: cipb.CompressedImage, t):
        nparr = np.frombuffer(msg.data, np.uint8)
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
            filename_matrix = f'{self.dir}/{self.name}_matrix_{w}x{h}.npy'
            filename_coeffs = f'{self.dir}/{self.name}_coeffs_{w}x{h}.npy'
            np.save(filename_matrix, mtx)
            np.save(filename_coeffs, dist)

            print("✅ Calibration réussie ! Fichiers enregistrés :")
            print(f"- {filename_matrix}")
            print(f"- {filename_coeffs}")
        else:
            print("⚠️ Pas assez d'images pour calibrer (au moins 10 bonnes prises nécessaires)")


if __name__ == "__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument('name', help='Camera name')
    parser.add_argument('-c', '--cam', type=int, help='Camera ID', default=None)
    parser.add_argument('-v', '--video', help='Video file', default=None)
    parser.add_argument('-t', '--topic', help='eCAL topic', default=None)
    parser.add_argument('-d', '--dir', default='../../drivers/camera/calibrations/', help='Directory for calibration files')
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
    
    calibrator = Calibrator(args.name, src_type, src)
    calibrator.run()
