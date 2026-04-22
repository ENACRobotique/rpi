#!/usr/bin/env python3
import cv2
import numpy as np
import os, sys
import argparse
from enum import Enum
import glob

# Paramètres du damier
NB_CORNERS_X = 9  # coins internes horizontalement
NB_CORNERS_Y = 6  # coins internes verticalement
SQUARE_SIZE_MM = 17.7  # taille réelle d’un carré en mm


if __name__ == "__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument('name', help='Camera name')
    parser.add_argument('basename', help='files basename')
    parser.add_argument('-s', '--scale', type=float, default=1)
    args = parser.parse_args()

    # Préparation des points 3D de référence (repère monde)
    objp = np.zeros((NB_CORNERS_Y * NB_CORNERS_X, 3), np.float32)
    objp[:, :2] = np.mgrid[0:NB_CORNERS_X, 0:NB_CORNERS_Y].T.reshape(-1, 2)
    objp *= SQUARE_SIZE_MM / 1000.0  # en mètres


    # Listes de points pour calibration
    objpoints = []  # points 3D
    imgpoints = []  # points 2D

    files = sorted(glob.glob(f"{args.basename}*.jpg"))

    for filepath in files:
        print(filepath)
        frame = cv2.imread(filepath)
        small = cv2.resize(frame, None, fx=args.scale, fy=args.scale)
        gray_small = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        ret_corners, corners = cv2.findChessboardCorners(gray_small, (NB_CORNERS_X, NB_CORNERS_Y), None)
        print(ret_corners)
        if ret_corners:
            cv2.drawChessboardCorners(small, (NB_CORNERS_X, NB_CORNERS_Y), corners, ret_corners)
            corners = corners / args.scale
            gray_full = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            
            cv2.cornerSubPix(
                gray_full,
                corners,
                winSize=(11, 11),
                zeroZone=(-1, -1),
                criteria=criteria
            )

            objpoints.append(objp)
            imgpoints.append(corners)
            imageSize = gray_full.shape[::-1]
        cv2.imshow('Calibration', small)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break


    if len(objpoints) >= 10:
        print("📐 Calibration en cours...")
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, imageSize, None, None)
        w, h = imageSize
        filename_calib = f'../../data/camera_calibrations/{args.name}_{w}x{h}.yml'
        fs = cv2.FileStorage(filename_calib, cv2.FILE_STORAGE_WRITE)
        fs.write("camera_matrix", mtx)
        fs.write("dist_coeffs", dist)
        fs.release()

        print("✅ Calibration réussie ! Fichier enregistré :")
        print(f"- {filename_calib}")
    else:
        print("⚠️ Pas assez d'images pour calibrer (au moins 10 bonnes prises nécessaires)")
