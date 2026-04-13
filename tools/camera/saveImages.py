#!/usr/bin/env python3
import cv2
import numpy as np
import os, sys
import argparse
from enum import Enum

"""
A good camera calibration needs:
    - fixed focus
    - the checkboard pattern should cover at least 20% of the image
    - the pattern should be at an angle less than 45° from the camera plane
    - capture the pattern at different orientations, and all over the image
"""

class ImageRecorder:
    def __init__(self, args) -> None:
        self.basename = args.basename
        self.count = 0

        self.init_capture(args.cam, args)


    def init_capture(self, src, args):
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
    
    def stop_capture(self):
        self.cap.release()
    
    def run(self):
        print("Appuie sur 'espace' pour capturer une image, 'q' pour quitter.")

        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            
            frame_disp = cv2.resize(frame, None, fx=args.scale, fy=args.scale)

            cv2.imshow('ImageRecorder', frame_disp)
            
            
            key = cv2.waitKey(1)

            if key == ord(' '):
                filename = f"{self.basename}_{self.count:02}.jpg"
                cv2.imwrite(filename, frame)
                print(f"{filename} saved !")
                self.count += 1

            elif key == ord('q'):
                break

        self.stop_capture()
        cv2.destroyAllWindows()
        

if __name__ == "__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument('cam', type=int, help='Camera ID', default=None)
    parser.add_argument('-W', '--width', type=int, help='image width, for camera', default=None)
    parser.add_argument('-H', '--height', type=int, help='image height, for camera', default=None)
    parser.add_argument('-f', '--fps', type=int, help='framerate', default=None)
    parser.add_argument('--fourcc', type=str, help='fourcc type (MJPG, H264, ...)', default=None)
    parser.add_argument('-s', '--scale', type=float, default=1)
    parser.add_argument('-b', '--basename', type=str, help='basename', default="image")

    args = parser.parse_args()

    calibrator = ImageRecorder(args)
    calibrator.run()
