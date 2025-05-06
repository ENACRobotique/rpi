#!/usr/bin/env python3
import cv2
import numpy as np
from sklearn.cluster import DBSCAN

import matplotlib.pyplot as plt
import sys
sys.path.append("../..")
import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber
from ecal.core.publisher import ProtoPublisher
from generated.robot_state_pb2 import Position

ARUCO_SIZE = 0.022  # 22 mm
TARGET_ARUCO_ID = 47 # Aruco ciblé
DIM = 500

class ArucoFinder:
    def __init__(self, camera):
        """Provide Camera ID"""
        if not ecal_core.is_initialized():
            ecal_core.initialize(sys.argv, "arucoFinder")
        
        self.conserve_pub = ProtoPublisher("Arucos", Position_aruco)
        
        self.visu = False
        self.camera_Id = camera
        # ArUco settings (API OpenCV 4.7+)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.arucoFound = None


    def getCalibration(self, matrix, coefs):
        """Provide Calibration Matrix and distance coefs as .npy file"""
        # Charger la calibration
        self.camera_matrix = np.load(matrix)
        self.dist_coeffs = np.load(coefs)

    def start(self):
        """Call once to start video capture"""
        # Capture vidéo
        self.cap = cv2.VideoCapture(self.camera_Id)
        
    def end(self):
        self.cap.release()
        cv2.destroyAllWindows()
        
    def visualize(self):
        # Fenêtre persistante de matplotlib
        if not self.visu:
            # plt.ion()
            # self.fig, self.ax = plt.subplots(figsize=(5, 5))
            cv2.namedWindow('ArUco Positioning', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('ArUco Positioning', 800, 600)
            self.visu = True
        # self.ax.clear()
        # # Actualiser la vue matplotlib
        # self.ax.set_title("Vue de dessus")
        # self.ax.set_xlabel("y (mm)")
        # self.ax.set_ylabel("x (mm)")
        # self.ax.set_xlim(-DIM/2, DIM/2)
        # self.ax.set_ylim(0, DIM)
        # self.ax.grid(True)

        # Dessin des marqueurs
        cv2.aruco.drawDetectedMarkers(self.frame, self.corners, self.ids)
        if self.ids is not None:
            for i in range(len(self.ids)):
                cv2.drawFrameAxes(self.frame, self.camera_matrix, self.dist_coeffs, self.rvecs[i], self.tvecs[i], 0.03)

        # # Cylindres visibles
        # if self.arucoFound is not None:
        #     xs,ys = self.arucoFound.x, self.arucoFound.y
        #     for i in range(len(xs)):
        #         self.ax.add_patch(plt.Circle((ys[i],xs[i]), 73/2, color='blue', alpha=0.5))


        # # Affichage image
        # if self.posCentre is not None:
        #     pos = f"x: {self.posCentre.x:.0f}, y: {self.posCentre.y:.0f} "
        #     angle = f"angle: {np.degrees(self.posCentre.theta):.1f}"
        #     cv2.putText(self.frame, pos, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        #     cv2.putText(self.frame, angle, (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        #     # Centre ligne
        #     self.ax.plot(self.posCentre.y, self.posCentre.x, 'gx', markersize=10, label='Centre ligne')

        #     # Direction (flèche entre les extrêmes)
        #     self.ax.arrow(self.cyl_positions[0][0]*1000, self.cyl_positions[0][2]*1000,
        #             self.dir_vector[0]*1000, self.dir_vector[2]*1000,
        #             head_width=0.01*1000, head_length=0.02*1000, fc='green', ec='green', label='Orientation')
        #     self.ax.plot(0, 0, 'ro', label='Camera')
        #     self.ax.legend()
        # plt.pause(0.001)

        # nb = f"Visible:{len(self.arucoFound)}"
        # cv2.putText(self.frame, nb, (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
        # Affichage image
        cv2.imshow("ArUco Positioning", self.frame)

    def centreConserve(self):
        if self.ids is not  None :
            
            if len(self.ids) > 0:
                
                # Filtrer la target ID
                # indices = [i for i, id_ in enumerate(self.ids) if id_[0] == TARGET_ARUCO_ID]
                # self.corners = np.array([self.corners[i] for i in indices])
                # self.ids = np.array([self.ids[i] for i in indices])
                
                # Estimation de la pose
                self.rvecs, self.tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(self.corners, ARUCO_SIZE, self.camera_matrix, self.dist_coeffs)

                # Extraction des positions 3D des ArUco
                if self.tvecs is not None:
                    points = np.array([tv[0] for tv in self.tvecs])  # [x, y, z] pour chaque ArUco

                    # Groupement en cylindres (selon x, z)
                    clustering = DBSCAN(eps=0.04, min_samples=1).fit(points[:, [0, 2]])
                    labels = clustering.labels_

                    # Moyenne des positions pour chaque cylindre
                    self.cyl_positions = []
                    for label in set(labels):
                        group = points[labels == label]
                        center = group.mean(axis=0)
                        self.cyl_positions.append(center)

                    self.cyl_positions = np.array(self.cyl_positions)


                    if len(self.cyl_positions) >= 2:
                        # Trier selon x
                        self.cyl_positions = self.cyl_positions[np.argsort(self.cyl_positions[:, 0])]

                        # Centre de la ligne des cylindres
                        x,y,z = self.cyl_positions.mean(axis=0)

                        # Orientation (angle)
                        self.dir_vector = self.cyl_positions[-1] - self.cyl_positions[0]
                        self.angle_to_line = np.pi-np.arctan2(self.dir_vector[0], self.dir_vector[2])
                        
                        self.posCentre = Position_aruco(x=z*1000,y=x*1000,theta=self.angle_to_line)

                        # Affichage console
                        print("\n--- Position du robot par rapport au centre de la ligne ---")
                        print(f"Position du centre : x = {self.posCentre.x:.2f} mm, y = {self.posCentre.y:.2f} mm")
                        print(f"Angle robot ↔ ligne : {np.degrees(self.posCentre.theta):.1f}°")

                        # self.conserve_pub.send(self.posCentre)
    
    def update(self):
        """Call it in a while true loop"""
        self.ret, self.frame = self.cap.read()
        if not self.ret:
            return
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)

        # Détection ArUco
        self.corners, self.ids, rejected = self.aruco_detector.detectMarkers(gray)
        # self.corners = np.array(self.corners)
        # self.ids = np.array(self.ids)

        if self.ids is not None :
            if len(self.ids) > 0:
                # Estimation de la pose
                self.rvecs, self.tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(self.corners, ARUCO_SIZE, self.camera_matrix, self.dist_coeffs)

                # Extraction des positions 3D des ArUco
                if self.tvecs is not None:
                    xs, ys, zs, ids, aruIds= [],[],[],[],[]
                    for i,tv  in enumerate(self.tvecs):
                        xs.append(tv[0][2]*1000) #to mm
                        ys.append(tv[0][0]*1000) #to mm
                        zs.append(-tv[0][1]*1000) #to mm
                        ids.append(i)
                        aruIds.append(self.ids[i][0])
                    self.arucoFound = Position_aruco(index = ids, x=xs, y=ys, z=zs, ArucoId = aruIds)
                    self.conserve_pub.send(self.arucoFound)


        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.end()
            return




if __name__ == "__main__":
    jesus = ArucoFinder(2)
    jesus.getCalibration('camera_matrix.npy','dist_coeffs.npy')
    jesus.start()
    while ecal_core.ok():
        jesus.update()
        jesus.visualize()

    