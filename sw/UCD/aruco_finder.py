#!/usr/bin/env python3
import cv2
import numpy as np
import sys, os

sys.path.append(os.path.join(os.path.dirname(__file__), '../..')) # Avoids ModuleNotFoundError when finding generated folder
import ecal.nanobind_core as ecal_core
from ecal.msg.proto.core import Publisher as ProtoPublisher
from generated.robot_state_pb2 import Aruco, Arucos
# from generated.robot_state_pb2 import Position_aruco
# from generated import CompressedImage_pb2 as cipb
import argparse
from enum import Enum
from scipy.spatial.transform import Rotation
#import time
from threading import Event


class Source(Enum):
    CAM = 0
    VIDEO = 1
    ECAL = 2
    

class ArucoFinder:
    def __init__(self, name, src_type, src, arucos, display):
        if not ecal_core.is_initialized():
         ecal_core.initialize("aruco_finder")
        
        self.name = name
        self.src_type = src_type
        self.src = src
        self.arucos = arucos
        self.display = display
        
        self.event = Event()
        self.img = None     # img received from eCAL

        # ArUco settings (API OpenCV 4.7+)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.world_objects = []   # dict id → position world

        self.camera_matrix = None
        self.dist_coeffs = None

        self.camera_pose_in_W = None
        self.camera_rot_in_W = None


        self.aruco_pub = ProtoPublisher(Arucos, "Arucos_world")
        
        self.open_capture()

    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_value, traceback):
        pass
    #     if src_type == Source.ECAL:
    #         self.sub.remove_receive_callback()
    
    def open_capture(self):
        if src_type == Source.CAM:
            self.cap = cv2.VideoCapture(src)
            if args.fourcc is not None:
                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*args.fourcc))
            if self.cap is None:
                print(f"Failed to open {self.name} with id {src}")
                exit(-1)
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
        # elif src_type == Source.ECAL:
        #     self.sub = ProtoSubscriber(cipb.CompressedImage, src)
        #     self.sub.set_receive_callback(self.on_img)

    def getCalibration(self, w, h):
        """Provide Calibration Matrix and distance coefs as .npy file"""
        # Charger la calibration
        f_calib = f'../../data/camera_calibrations/{self.name}_{w}x{h}.yml'
        fs = cv2.FileStorage(f_calib, cv2.FILE_STORAGE_READ)
        self.camera_matrix = fs.getNode("camera_matrix").mat()
        self.dist_coeffs = fs.getNode("dist_coeffs").mat()
        fs.release()
    
    # def on_img(self, pub_id: ecal_core.TopicId, data: ReceiveCallbackData[cipb.CompressedImage]):
    #     nparr = np.frombuffer(data.message.data, np.uint8)
    #     self.img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    #     self.event.set()

    # def send_processed_frame(self, frame):
    #     img_encode = cv2.imencode(".jpg", frame)[1]
    #     byte_encode = img_encode.tobytes()
    #     ci = cipb.CompressedImage(timestamp=Timestamp(), data=byte_encode, format='jpeg')
    #     self.cam_pub.send(ci)

    def estimatePoseSingleMarkers(self,corners, marker_size, mtx, distortion):
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
            ok, rvec, tvec = cv2.solvePnP(
                marker_points,
                c,
                mtx,
                distortion,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )

            trash.append(ok)

            # --- reshape to match OpenCV format ---
            rvec = rvec.reshape(1, 3)
            tvec = tvec.reshape(1, 3)

            rvecs.append(rvec)
            tvecs.append(tvec)

        # Convert lists to arrays: final shapes (N,1,3)
        rvecs = np.array(rvecs, dtype=np.float32)
        tvecs = np.array(tvecs, dtype=np.float32)

        return rvecs, tvecs, trash
    
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
                                    [600, 1400, 0],   # marqueur 20
                                    [2400, 1400, 0],  # marqueur 21
                                    [600, 600, 0],    # marqueur 22
                                    [2400, 600, 0]    # marqueur 23
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

        #print("Camera position (world):")
        #print(cam_pos_world)
        #print("Camera rotation (world):")
        #print(cam_rot_world)

        self.camera_pose_in_W = cam_pos_world.flatten()    # (3,)
        self.camera_rot_in_W  = cam_rot_world  

        self.cam_pos_known = True

        return cam_pos_world, cam_rot_world
    
    

    def get_camera_pose(self, frame):
        """Call it in a while true loop"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Détection ArUco
        detected_corners, detected_ids, rejected = self.aruco_detector.detectMarkers(gray)

        if self.display:
            cv2.aruco.drawDetectedMarkers(frame, detected_corners, detected_ids)
        
        cv2.aruco.drawDetectedMarkers(frame, detected_corners, detected_ids)

        if detected_corners:
            centers = []
            centers_id = []
            for corners, id in zip(detected_corners, detected_ids):
                id = id[0]
                if id not in [20,21,22,23]:
                    continue
                center = corners[0].mean(axis=0)
                centers.append(center)
                centers_id.append(id)

                size = self.arucos[id]
            
            combined = list(zip(centers_id, centers))

            # Sort by id
            combined.sort(key=lambda x: x[0])
            centers_id, centers = zip(*combined)

            centers_id = list(centers_id)
            centers = list(centers)

            if centers :        
                rvecs, tvecs, _ = self.estimatePoseFromCenters(centers, self.camera_matrix, self.dist_coeffs)
 
                if tvecs is not None:
                    rv, tv = rvecs[0], tvecs[0]
                    
                    self.camera_pose_in_world_from_tags(rv, tv)

                    cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rv, tv, size)
                   
        return frame
    

    def draw_world_map(self):
        """
        Carte monde top-down :
        - table fixe
        - ArUco à l'échelle réelle
        - caméra à l'échelle réelle
        - RIEN ne sort de la table
        """

        if not hasattr(self, "world_objects") or len(self.world_objects) == 0:
            return

        # ============================
        # PARAMÈTRES MONDE
        # ============================

        TABLE_X = 3000  # mm
        TABLE_Y = 2000  # mm

        # Centre monde = centre table
        cx = TABLE_X / 2
        cy = TABLE_Y / 2

        # ============================
        # IMAGE
        # ============================

        img_size = 800
        margin = 40  #

        map_img = np.ones((img_size, img_size, 3), dtype=np.uint8) * 255

        px_per_mm_x = (img_size - 2 * margin) / TABLE_X
        px_per_mm_y = (img_size - 2 * margin) / TABLE_Y
        px_per_mm = min(px_per_mm_x, px_per_mm_y)

        # Origine graphique 
        ox = img_size // 2
        oy = img_size // 2

        # ============================
        # 1) DESSIN TABLE
        # ============================

        table_corners = [
            (0, 0),
            (TABLE_X, 0),
            (TABLE_X, TABLE_Y),
            (0, TABLE_Y)
        ]

        table_pts_img = []
        for xw, yw in table_corners:
            xi = int(ox + (xw - cx) * px_per_mm)
            yi = int(oy - (yw - cy) * px_per_mm)
            table_pts_img.append((xi, yi))

        cv2.polylines(
            map_img,
            [np.array(table_pts_img, dtype=np.int32)],
            True,
            (255, 0, 0),
            3
        )

        # draw aruco

        for obj in self.world_objects:
            pos = obj["pos"]    # centre (mm)
            size = obj["size"]  # taille réelle (mm)
            aruco_id = obj["id"]

            xw, yw = pos[0], pos[1]

            xi = int(ox + (xw - cx) * px_per_mm)
            yi = int(oy - (yw - cy) * px_per_mm)

            half_px = int((size / 2) * px_per_mm)

            # Dessin du carré
            cv2.rectangle(
                map_img,
                (xi - half_px, yi - half_px),
                (xi + half_px, yi + half_px),
                (0, 0, 255),
                2
            )

            # Texte ID
            cv2.putText(
                map_img,
                f"ID {aruco_id}",
                (xi + half_px + 5, yi),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 0),
                1
            )

        # ============================
        # 3) DESSIN CAMÉRA (À L’ÉCHELLE)
        # ============================

        if hasattr(self, "camera_pose_in_W") and self.camera_pose_in_W is not None:
            #print(self.camera_pose_in_W)
            cam_x = self.camera_pose_in_W[0]
            cam_y = self.camera_pose_in_W[1]

            xi = int(ox + (cam_x - cx) * px_per_mm)
            yi = int(oy - (cam_y - cy) * px_per_mm)

            cam_radius_mm = 50  
            cam_radius_px = int(cam_radius_mm * px_per_mm)

            cv2.circle(map_img, (xi, yi), cam_radius_px, (0, 0, 0), -1)
            cv2.putText(
                map_img,
                "CAM",
                (xi + cam_radius_px + 5, yi),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 0),
                1
            )

            if hasattr(self, "camera_rot_in_W"):
                R = self.camera_rot_in_W
                cam_x_axis = R[:, 0]

                L_mm = 300
                dx = int(cam_x_axis[0] * L_mm * px_per_mm)
                dy = int(cam_x_axis[1] * L_mm * px_per_mm)

                cv2.arrowedLine(
                    map_img,
                    (xi, yi),
                    (xi + dx, yi - dy),
                    (0, 150, 0),
                    2,
                    tipLength=0.2
                )

        # ============================
        # AFFICHAGE
        # ============================

        cv2.imshow("World Map (Metric, Fixed)", map_img)



    def process(self, frame):
        """Call it in a while true loop"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


        
        
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        gray = clahe.apply(gray)

        # Détection ArUco
        detected_corners, detected_ids, rejected = self.aruco_detector.detectMarkers(gray)

        if self.display:
            cv2.aruco.drawDetectedMarkers(frame, detected_corners, detected_ids)
            self.world_objects.clear()
        
        cv2.aruco.drawDetectedMarkers(frame, detected_corners, detected_ids)

        if detected_corners:
            arucos = []
            for corners, id in zip(detected_corners, detected_ids):
                id = id[0]
            
                if id not in self.arucos:
                    continue
                size = self.arucos[id]
            
                rvecs, tvecs, _ = self.estimatePoseSingleMarkers(corners, size, self.camera_matrix, self.dist_coeffs)
 
                if tvecs is not None:
                    rv, tv = rvecs[0], tvecs[0]

                    #posW = self.objects_in_world(rv, tv)
                    if self.display:
                        cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rv, tv, size)
     
                    # Convert rvec to rotation matrix
                    P_tc = np.array(tv[0])
                    P_cw = np.array(self.camera_pose_in_W)


                    R_wc = self.camera_rot_in_W   #Rotation.from_quat(Q_wc)
                    P_tw = R_wc @ P_tc + P_cw

                    # Rotation marqueur -> caméra
                    R_ct, _ = cv2.Rodrigues(rv)

                    # Rotation marqueur -> monde
                    R_wt = R_wc @ R_ct

                    # Convertir en quaternion
                    r = Rotation.from_matrix(R_wt)
                    (qx, qy, qz, qw) = r.as_quat(False)

                    ar = Aruco(x=P_tw[0], y=P_tw[1], z=P_tw[2],qx=qx, qy=qy, qz=qz, qw=qw, ArucoId=id )
                    arucos.append(ar)

                    # TODO : modify the world map to use the arucos message instead
                    if self.display:
                        self.world_objects.append( {
                            "pos": P_tw,
                            "size": size,
                            "id": id
                        })
                
            self.arucoFound = Arucos(arucos=arucos, cameraName=self.name)
            self.aruco_pub.send(self.arucoFound)
        return frame
    

    def run(self):
        win_name = f"ArucoFinder - {self.name}"
        if self.display :
            cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)

        while True:
            if self.src_type == Source.CAM or self.src_type == Source.VIDEO:
                ret, frame = self.cap.read()
            else:
                self.event.wait()
                frame = self.img.copy()
            
            if self.camera_matrix is None:
                h, w, _ = frame.shape
                self.getCalibration(w, h)
            
            if self.camera_pose_in_W is None:
                calibration_frame = self.get_camera_pose(frame)

            if self.camera_pose_in_W is not None:
                processed = self.process(frame)
            # if self.display:
            #     self.send_processed_frame(processed)

                if self.display :

                    self.draw_world_map()

            if self. display:
                if self.camera_pose_in_W is not None:
                    cv2.imshow(f"ArucoFinder - {self.name}", processed)
                else :
                    cv2.imshow(f"ArucoFinder - {self.name}", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


if __name__ == "__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument('name', help='camera name')
    parser.add_argument('-c', '--cam', type=int, help='Camera ID', default=None)
    parser.add_argument('-v', '--video', help='Video file', default=None)
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

    else:
        print("Please specify the source: cam, video or ecal topic.")
    
    arucos = {20:100, 21:100, 22:100, 23:100, 6:70, 47:30, 13:30, 36:30, 7:70}

    known_markers = {
        20: np.array([600, 1400, 0]),   # x=600, y=1400, z=0
        21: np.array([2400, 1400, 0]),  # x=2400, y=1400, z=0
        22: np.array([600, 600, 0]),    # x=600, y=600, z=0
        23: np.array([2400, 600, 0])    # x=2400, y=600, z=0
    }

    with ArucoFinder(args.name, src_type, src, arucos, args.display) as af:
        af.run()#k