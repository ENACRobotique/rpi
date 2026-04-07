#!/usr/bin/env python3
import cv2
import numpy as np
import sys, os

import ecal.nanobind_core as ecal_core
from ecal.msg.proto.core import Subscriber as ProtoSubscriber
from ecal.msg.common.core import ReceiveCallbackData
from generated import CompressedImage_pb2 as cipb

sys.path.append(os.path.join(os.path.dirname(__file__), '../..')) # Avoids ModuleNotFoundError when finding generated folder
from threading import Event
import argparse
from enum import Enum
from scipy.spatial.transform import Rotation


# Paramètres du damier
NB_CORNERS_X = 9  # coins internes horizontalement
NB_CORNERS_Y = 6  # coins internes verticalement
SQUARE_SIZE_MM = 17.7  # taille réelle d’un carré en mm

CHESS_THICKNESS = 3 # epaisseur du damier en mm




class Source(Enum):
    CAM = 0
    VIDEO = 1
    ECAL = 2


class Calibrator2000:
    def __init__(self, name, src_type, src):

        
        self.name = name
        self.src_type = src_type
        self.src = src
        self.dir = args.dir
        self.x_damier = args.x
        self.y_damier = args.y
    

        self.camera_matrix = None
        self.dist_coeffs = None

        self.camera_pose_in_W = None
        self.camera_rot_in_W = None

        self.event = Event()
        self.imageSize = (0, 0)
        self.img: np.ndarray = None
        
        self.open_capture()

    
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
            if not ecal_core.is_initialized():
                ecal_core.initialize("arucoFinder")
            self.sub = ProtoSubscriber(cipb.CompressedImage, src)
            self.sub.set_receive_callback(self.on_img)

    
    def on_img(self, pub_id: ecal_core.TopicId, data: ReceiveCallbackData[cipb.CompressedImage]):
        nparr = np.frombuffer(data.message.data, np.uint8)
        frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        self.img = frame
        self.event.set()
        


    def getCalibration(self, w, h):
        """Provide Calibration Matrix and distance coefs as .npy file"""
        # Charger la calibration
        f_calib = f'{self.dir}/{self.name}_{w}x{h}.yml'
        fs = cv2.FileStorage(f_calib, cv2.FILE_STORAGE_READ)
        self.camera_matrix = fs.getNode("camera_matrix").mat()
        self.dist_coeffs = fs.getNode("dist_coeffs").mat()
        fs.release()
    

    
    def estimatePoseFromCenters(self, centers, mtx, distortion):
        """
        Estimate camera pose from 4 points (centers of 4 ArUco) and their positions in world coordinates.

        Args:
            centers (list of np.ndarray): liste des centres détectés dans l'image, shape (4,2)
            world_positions (list of np.ndarray): liste des positions connues des marqueurs, shape (4,3)
            mtx (np.ndarray): matrice intrinsèque caméra
            distortion (np.ndarray): coefficients de distorsion caméra

        Returns:
            rvec (np.ndarray): vecteur rotation (3x1)
            tvec (np.ndarray): vecteur translation (3x1)
            ok (bool): True si solvePnP a réussi
        """
        object_points = np.array([
                                    [-4*SQUARE_SIZE_MM, 2.5*SQUARE_SIZE_MM, 0],   # marker 20
                                    [4*SQUARE_SIZE_MM, 2.5*SQUARE_SIZE_MM, 0],  # marker 21
                                    [4*SQUARE_SIZE_MM, -2.5*SQUARE_SIZE_MM, 0],    # marker 22
                                    [-4*SQUARE_SIZE_MM, -2.5*SQUARE_SIZE_MM, 0]    # marker 23
                                ], dtype=np.float32)
        if len(centers) < 4:
            print("Pas assez de points pour solvePnP")
            return None, None, False

        # Convertir en np.array float32
        image_points = np.array(centers, dtype=np.float32)

        rvecs = []
        tvecs = []

        # SolvePnP
        ok, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            mtx,
            distortion,
            flags=cv2.SOLVEPNP_ITERATIVE
        )

        if ok:
            rvec = rvec.reshape(1, 3)
            tvec = tvec.reshape(1, 3)
        

        rvecs.append(rvec)
        tvecs.append(tvec)

        return rvecs, tvecs, ok
    

    def camera_pose_in_world_from_tags(self, rvec, tvec):
        """
        Compute the camera pose in the world coordinate frame,
        assuming the ArUco marker frame is perfectly aligned with the world frame.

        Args:
            rvec (np.ndarray): Rotation vector (3x1) describing the marker pose 
                            in the camera frame (marker → camera).
            tvec (np.ndarray): Translation vector (3x1) describing the marker pose 
                            in the camera frame (marker → camera).

        Returns:
            cam_pos_world (np.ndarray): Camera position in the world frame (3x1).
            cam_rot_world (np.ndarray): Camera rotation matrix in the world frame (3x3).
        """

        # --- Convert rotation vector into rotation matrix ---
        # R_marker_cam: rotation from marker frame to camera frame
        R_marker_cam, _ = cv2.Rodrigues(rvec)

        # --- Compute camera position in world frame ---
        # Inverting the transform:
        # Camera position in marker frame is: -R^T * t
        # Since marker frame ≡ world frame, this is also the camera position in world coordinates.
        cam_pos_world = - R_marker_cam.T @ tvec.reshape(3, 1)

        # --- Compute camera orientation in world frame ---
        # R_marker_cam: marker → camera
        # Therefore R_cam_marker = R_marker_cam.T
        #
        # Since marker frame is perfectly aligned with world frame:
        # R_cam_world = R_cam_marker
        cam_rot_world = R_marker_cam.T


        self.camera_pose_in_W = cam_pos_world.flatten()  + np.array([self.x_damier , self.y_damier, CHESS_THICKNESS])  # (3,)
        self.camera_rot_in_W  = cam_rot_world  


        self.cam_pos_known = True

        return cam_pos_world, cam_rot_world
    



    def pos_damier(self,frame):
        """
    
         récupère les 4 coins extrieurs pour déterminer la position de la caméra 
       
         """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret_corners, corners = cv2.findChessboardCorners(gray, (NB_CORNERS_X, NB_CORNERS_Y), None)
        if ret_corners:

            if ret_corners:
                top_left = corners[0]
                top_right = corners[NB_CORNERS_X - 1]
                bottom_left = corners[(NB_CORNERS_Y - 1) * NB_CORNERS_X]
                bottom_right = corners[NB_CORNERS_X * NB_CORNERS_Y - 1]

                ext_corners = [top_left, top_right, bottom_right, bottom_left]



                # affichage
                for i, c in enumerate(ext_corners):
                    x, y = c.ravel().astype(int)
                    cv2.circle(frame, (x, y), 10, (0, 0, 255), -1)
                    cv2.putText(frame, str(i), (x,y -5), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0),  3)

                
            if ext_corners :        
                rvecs, tvecs, _ = self.estimatePoseFromCenters(ext_corners, self.camera_matrix, self.dist_coeffs)


                if tvecs is not None:
                    rv, tv = rvecs[0], tvecs[0]
                    
                    self.camera_pose_in_world_from_tags(rv, tv)

                    cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rv, tv, 2*SQUARE_SIZE_MM)


                    # Convert rvec to rotation matrix
                    rotation_matrix, _ = cv2.Rodrigues(np.array(rv))
                    r =  Rotation.from_matrix(rotation_matrix)
                    (qx, qy, qz, qw) = r.as_quat()

            #self.arucoFound = Position_aruco(x=xs, y=ys, z=zs, qx=qxs, qy=qys, qz=qzs, qw=qws, ArucoId=aruIds, cameraName=self.name)
            #self.aruco_pub.send(self.arucoFound)
                

        cv2.imshow('Calibrator2000', frame)
            

            

        #cv2.destroyAllWindows()



    def run(self):

        while True:

            if src_type == Source.CAM or src_type == Source.VIDEO:
                ret, frame = self.cap.read()
                if not ret:
                    break
            elif src_type == Source.ECAL:
                self.event.wait()
                frame = self.img.copy()

            
            if self.camera_matrix is None:
                h, w, _ = frame.shape
                self.getCalibration(w, h)
            
            self.pos_damier(frame)
            
            # if self.camera_pose_in_W is None:
            #     calibration_frame = self.get_camera_pose(frame)

            # processed = self.process(frame)
            # if self.display:
            #     self.send_processed_frame(processed)

            
            # cv2.imshow(f"ArucoFinder - {self.name}", processed)

            key = cv2.waitKey(1) 

            if key == ord(" ") and self.camera_pose_in_W is not None and self.camera_rot_in_W is not None :

                filename_tvec = f'{self.dir}/{self.name}_tvec.npy'
                filename_rvec = f'{self.dir}/{self.name}_rvec.npy'
                np.save(filename_tvec, self.camera_pose_in_W)
                np.save(filename_rvec, self.camera_rot_in_W)

                print(f'✅ Position de la camera enregistrée dans "{self.dir}/{self.name}_..." ❤️ ')

            elif key == ord('q'):
                break


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description='Calbration of camera pos with respect to the centers of the calibration chessboard')
    parser.add_argument('name', help='camera name')
    parser.add_argument('-c', '--cam', type=int, help='Camera ID', default=None)
    parser.add_argument('-v', '--video', help='Video file', default=None)
    parser.add_argument('-t', '--topic', help='ecal', default=None)
    parser.add_argument('-W', '--width', type=int, help='image width', default=None)
    parser.add_argument('-H', '--height', type=int, help='image height', default=None)
    parser.add_argument('-f', '--fps', type=int, help='framerate', default=None)
    parser.add_argument('-d', '--dir', default='../../data/camera_calibrations/', help='Directory for calibration files')
    parser.add_argument('--fourcc', type=str, help='fourcc type (MJPG, H264, ...)', default=None)

    parser.add_argument('-x', type=float, help='X_damier/robot', default=0)
    parser.add_argument('-y', type=float, help='Y_damier/robot', default=0)
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
        print("Please specify the source: cam, video ")
    


    calibrator =  Calibrator2000(args.name, src_type, src )
    calibrator.run()