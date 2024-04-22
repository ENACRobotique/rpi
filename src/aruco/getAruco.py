import cv2
import math
import numpy as np
from pyquaternion import Quaternion
import time

import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher
import sys
import generated.robot_state_pb2 as hlm

ROUE = 3 # Numéro du servo de la roue
BRAS = 1 # identique pour le bras
BRAS_BAS = 1960
BRAS_HAUT = 900

# load the ArUCo dictionary and grab the ArUCo parameters
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
arucoParams = cv2.aruco.DetectorParameters()
aruco_detector = cv2.aruco.ArucoDetector()
aruco_detector.setDictionary(arucoDict)
aruco_detector.setDetectorParameters(arucoParams)
cameraMatrix = np.array([[499.14976028 ,  0. ,        324.30564055],
 [  0.         ,498.93850218 ,242.0044319 ],
 [  0.           ,0.           ,1.        ]])
dist = np.array([[ 1.70293952e-01 ,-3.29376421e-01  ,1.79606916e-03  ,2.15408616e-04
   ,7.75206391e-02]])

def find_quat(rvecs):
    a = np.array(rvecs[0])
    theta = math.sqrt(a[0]**2 + a[1]**2 + a[2]**2) 
    b = a/theta 
    qx = b[0] * math.sin(theta/2)
    qy = -b[1] * math.sin(theta/2) # left-handed vs right handed
    qz = b[2] * math.sin(theta/2)
    qw = math.cos(theta/2)
    return (qw, qx, qy, qz)

def start_video():
    return cv2.VideoCapture(0)

def estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    '''
    This will estimate the rvec and tvec for each of the marker corners detected by:
       corners, ids, rejectedImgPoints = detector.detectMarkers(image)
    corners - is an array of detected corners for each detected marker in the image
    marker_size - is the size of the detected markers
    mtx - is the camera matrix
    distortion - is the camera distortion matrix
    RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
    '''
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    trash = []
    rvecs = []
    tvecs = []
    
    for c in corners:
        nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(R)
        tvecs.append(t)
        trash.append(nada)
    return rvecs, tvecs, trash


def end_video(vs):
    vs.stop()

def get_Aruco(vs):
    ret , frame = vs.read()
    frame = cv2.undistort(frame,cameraMatrix,dist,None)
    # detect ArUco markers in the input frame
    (corners, ids, rejected) = aruco_detector.detectMarkers(frame)
    # verify *at least* one ArUco marker was detected
    if len(corners) > 0:
        # flatten the ArUco IDs list
        ids = ids.flatten()
        # loop over the detected ArUCo corners
        for (markerCorner, markerID) in zip(corners, ids):
            if markerID != 47: # Ne prend en compte que les ArUCo des panneaux solaires
                continue
            # extract the marker corners (which are always returned
            # in top-left, top-right, bottom-right, and bottom-left
            # order)
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            # convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            rvecs, tvecs, objPoints = estimatePoseSingleMarkers(markerCorner,37,cameraMatrix,dist)
            quat = Quaternion(find_quat(rvecs))
            rad = list(map(lambda item: round(item,2) ,quat.yaw_pitch_roll))

            return rad[0] , tvecs  # yaw, center

def convert_deg_to_servo(deg): #FIXME le rendre adaptatif , change à chaque fois que le support est bougé
    ratio = 1
    return ratio * deg



if __name__ == "__main__":
    vs = start_video()
    ecal_core.initialize(sys.argv, "aruco")
    publisher = ProtoPublisher("aruco",hlm.Position_aruco)

    time.sleep(1)
    while True:
        a = get_Aruco(vs)
        if a != None :
            yaw_rad, center = a
            yaw = np.rad2deg(yaw_rad) + 68
            # print(f"yaw: {yaw}")
            # Passage du center de la base camera vers la base "bras"
            rot_x = np.deg2rad(-60) # en radians
            rot_y = np.deg2rad(-19.5)
            rot_z = 0
            cx , sx = math.cos(rot_x), math.sin(rot_x)
            rot_matrice_x = np.array([[1,0,0],[0,cx,-sx],[0,sx,cx]])
            cy , sy = math.cos(rot_y), math.sin(rot_y)
            rot_matrice_y = np.array([[cy,0,sy],[0,1,0],[-sy,0,cy]])
            cz , sz = math.cos(rot_z), math.sin(rot_z)
            rot_matrice_z = np.array([[cz,-sz,0],[sz,cz,0],[0,0,1]])
            center = np.matmul(rot_matrice_z,np.matmul(rot_matrice_y,np.matmul(rot_matrice_x,center)))[0]
            center[0] += 47
            center[1] += 0
            center[2] += 23.5

            message = hlm.Position_aruco()
            message.x = center[0]
            message.y = center[1]
            message.z = center[2]
            message.theta = yaw
            publisher.send(message)
            

    vs.release()


    # commande_angle = yaw + 68 # 104 = - valeur de l'angle par la camera
    # if commande_angle > 180:
    #     commande_angle = commande_angle - 360
    # if commande_angle < -180:
    #     commande_angle = commande_angle + 360
    # print(f"Deg_difference: {commande_angle}")

    

    # # Initialisation du servo roue

    # ser.write(f'servo {ROUE} 1500\r\n'.encode())
    # time.sleep(2)

    # # Descente du bras
    # ser.write(f'servo {BRAS} {BRAS_BAS}\r\n'.encode())
    # time.sleep(1)

    # # Tourne le panneau
    # print(f"commande: {convert_deg_to_servo(commande_angle)}")
    # ser.write(f'servo {ROUE} {1500 + convert_deg_to_servo(commande_angle)}\r\n'.encode())
    # time.sleep(3)

    # # Remonte le bras

    # ser.write(f'servo {BRAS} {BRAS_HAUT}\r\n'.encode())


