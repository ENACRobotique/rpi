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
class ArucoFinder:
    def __init__(self, cameraId, name):
        """Provide Camera ID"""
        if not ecal_core.is_initialized():
            ecal_core.initialize(sys.argv, "arucoFinder")
        
        self.aruco_pub = ProtoPublisher("Arucos", Position_aruco)
        self.cam_pub = ProtoPublisher("images_"+str(name), cipb.CompressedImage)
        self.camera_Id = cameraId
        self.name = name
        # ArUco settings (API OpenCV 4.7+)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.arucoFound = None
        self.arucosInUse = {}
        self.resolution = (640, 480) #width, height
        self.visu_to_ecal = False

    def getCalibration(self, matrix, coefs, resolution = (640,480)):
        """Provide Calibration Matrix and distance coefs as .npy file"""
        # Charger la calibration
        self.camera_matrix = np.load(matrix)
        self.dist_coeffs = np.load(coefs)
        self.resolution = resolution

    def start(self, arucosToUse:dict):
        """Call once to start video capture\n
        arucosToUse :{Aruco id : size in meters}
        """
        # Capture vidéo
        self.arucosInUse = arucosToUse
        self.cap = cv2.VideoCapture(self.camera_Id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
        w, h = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH), self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        print(f"Opened camera with resolution {w}x{h}!\n")

        
    def init_visu(self,w=640,h=480, visu_mode=0):
        
        if visu_mode == 2:
            self.visu_to_ecal = True
        
        else :
            cv2.namedWindow("ArUco Positioning", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("ArUco Positioning", w, h)
        
    def end(self):
        self.cap.release()
        cv2.destroyAllWindows()
    
    def visualize(self):
        if self.corners:
            if not self.visu_to_ecal:
                cv2.aruco.drawDetectedMarkers(self.frame, self.corners, self.ids)
        
        w, h = self.resolution[0],self.resolution[1]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeffs, (w,h), 0, (w,h))
        
        # undistort
        dst = cv2.undistort(self.frame, self.camera_matrix, self.dist_coeffs, None, newcameramtx)
        
        # crop the image
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
        if self.visu_to_ecal:
            img_encode = cv2.imencode(".jpg", self.frame)[1]
            byte_encode = img_encode.tobytes()
            #timestamp = Timestamp()
            ci = cipb.CompressedImage(timestamp=Timestamp(), data=byte_encode, format='jpeg')
            self.cam_pub.send(ci)
        else:
            cv2.imshow("ArUco Positioning", dst)
    
    def update(self):
        """Call it in a while true loop"""
        self.ret, self.frame = self.cap.read()
        if not self.ret:
            return
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)

        # Détection ArUco
        self.corners, self.ids, rejected = self.aruco_detector.detectMarkers(gray)
        
        if self.ids is not None :
            if len(self.ids) > 0:
                xs, ys, zs, ids, aruIds= [],[],[],[],[]
                # On cherche tout les couples aruco/taille voulu
                for aruco_id, size in self.arucosInUse.items():
                    indices = [i for i, id_ in enumerate(self.ids) if id_[0] == aruco_id]
                    # Estimation de la pose
                    if indices:
                        selected_corners = [self.corners[i] for i in indices]
                        self.rvecs, self.tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(selected_corners, size, self.camera_matrix, self.dist_coeffs)

                        # Extraction des positions 3D des ArUco
                        if self.tvecs is not None:
                            for i,tv  in enumerate(self.tvecs):
                                xs.append(tv[0][2]*1000) #to mm
                                ys.append(tv[0][0]*1000) #to mm
                                zs.append(-tv[0][1]*1000) #to mm
                                ids.append(i)
                                aruIds.append(self.ids[i][0])
                self.arucoFound = Position_aruco(index = ids, x=xs, y=ys, z=zs, ArucoId = aruIds, cameraName = self.name)
                self.aruco_pub.send(self.arucoFound)
                nb = f"Visible:{len(ids)}"
                cv2.putText(self.frame, nb, (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.end()
            return




if __name__ == "__main__":
    
    if len(sys.argv) < 2:
        print("Usage: ./arucoFinder.py <nb> <camera_name> <visu> <nb2> <cam_name2> <visu2>")
        exit(1)
    name = sys.argv[2]
    try:
        name2 = sys.argv[5]
    except IndexError:
        name2 = name
    visu = 0
    visu2 = 0
    try:
        visu = int(sys.argv[3])
        if visu:
            print("Visualization on")
    except IndexError:
        pass
    try:
        visu2 = int(sys.argv[6])
        if visu2:
            print("Visualization on")
    except IndexError:
        pass
    try:
        cam_nb =int(sys.argv[1])
    except ValueError:
        cam_nb = sys.argv[1]
    try:
        cam_nb2 =int(sys.argv[4])
    except ValueError:
        cam_nb2 = sys.argv[4]
    except IndexError:
        cam_nb2 = cam_nb

    dip, mab = False, False
    if name == 'dipper':
        dip = True
    if name2 == 'mabel':
        mab = True
    if name == 'both':
        dip, mab = True, True

    if dip:
        print("Starting dipper")
        dipper = ArucoFinder(cam_nb, name)
        dipper.getCalibration('dipper_matrix_1920x1080.npy','dipper_coeffs_1920x1080.npy', (1920,1080))
        dipper.start({47:0.022})
        if visu:
            dipper.init_visu(1920,1080,visu)
    if mab :
        print("Starting mabel")
        mabel = ArucoFinder(cam_nb2, name2)
        mabel.getCalibration('mabel_matrix_1920x1080.npy','mabel_coeffs_1920x1080.npy', (1920,1080))
        mabel.start({47:0.022})
        if visu2:
            mabel.init_visu(1920,1080,visu2)
    
    if not mab and not dip:
        print("Unregistered camera")
        print("Registerd cameras are 'dipper' 'mabel'")
    
    while ecal_core.ok():
        if dip :
            dipper.update()
            if visu:
                dipper.visualize()
        if mab:
            mabel.update()
            if visu2:
                mabel.visualize()