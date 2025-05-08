#!/usr/bin/env python3
import cv2
import numpy as np
import sys
sys.path.append("../..")
import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber
from ecal.core.publisher import ProtoPublisher
from generated.robot_state_pb2 import Position_aruco

class ArucoFinder:
    def __init__(self, cameraId, name):
        """Provide Camera ID"""
        if not ecal_core.is_initialized():
            ecal_core.initialize(sys.argv, "arucoFinder")
        
        self.aruco_pub = ProtoPublisher("Arucos", Position_aruco)
        self.camera_Id = cameraId
        self.name = name
        # ArUco settings (API OpenCV 4.7+)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.arucoFound = None
        self.arucosInUse = {}

    def getCalibration(self, matrix, coefs):
        """Provide Calibration Matrix and distance coefs as .npy file"""
        # Charger la calibration
        self.camera_matrix = np.load(matrix)
        self.dist_coeffs = np.load(coefs)

    def start(self, arucosToUse:dict):
        """Call once to start video capture\n
        arucosToUse :{Aruco id : size in meters}
        """
        # Capture vidéo
        self.arucosInUse = arucosToUse
        self.cap = cv2.VideoCapture(self.camera_Id)
        
    def end(self):
        self.cap.release()
        cv2.destroyAllWindows()
    
    def visualize(self):
        cv2.aruco.drawDetectedMarkers(self.frame, self.corners, self.ids)
        cv2.imshow("ArUco Positioning", self.frame)
    
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
    
    dipper = ArucoFinder(0, "dipper")
    dipper.getCalibration('dipper_matrix.npy','dipper_coeffs.npy')
    dipper.start({47:0.022})

    mabel = ArucoFinder(2, "mabel")
    mabel.getCalibration('mabel_matrix.npy','mabel_coeffs.npy')
    mabel.start({47:0.022})

    while ecal_core.ok():
        dipper.update()
        mabel.update()
       # dipper.visualize()
       # mabel.visualize()

    
