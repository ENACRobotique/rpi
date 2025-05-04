#!/usr/bin/env python3
import cv2
import numpy as np
from sklearn.cluster import DBSCAN

import matplotlib.pyplot as plt

# Fenêtre persistante de matplotlib
plt.ion()
fig, ax = plt.subplots(figsize=(5, 5))
ax.set_title("Vue de dessus")
ax.set_xlabel("x (m)")
ax.set_ylabel("z (m)")
dim = 500
ax.set_xlim(-dim/2, dim/2)
ax.set_ylim(0, dim)
sc = None
arrow = None
center_dot = None
robot_dot = ax.plot(0, 0, 'ro', label='Caméra')[0]
ax.grid(True)

TARGET_ID = 47 # Aruco ciblé

# Charger la calibration
camera_matrix = np.load('camera_matrix.npy')
dist_coeffs = np.load('dist_coeffs.npy')
marker_length = 0.022  # 22 mm

# ArUco settings (API OpenCV 4.7+)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# Capture vidéo
cap = cv2.VideoCapture(2)
cv2.namedWindow('ArUco Positioning', cv2.WINDOW_NORMAL)
cv2.resizeWindow('ArUco Positioning', 800, 600)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Détection ArUco
    corners, ids, rejected = aruco_detector.detectMarkers(gray)

    if ids is not None and len(ids) > 0:
        
        # Filtrer la target ID
        indices = [i for i, id_ in enumerate(ids) if id_[0] == TARGET_ID]
        corners = np.array([corners[i] for i in indices])
        ids = np.array([ids[i] for i in indices])
        
        # Estimation de la pose
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)

        # Dessin des marqueurs
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        for i in range(len(ids)):
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.03)

        # Extraction des positions 3D des ArUco
        points = np.array([tv[0] for tv in tvecs])  # [x, y, z] pour chaque ArUco

        # Groupement en cylindres (selon x, z)
        clustering = DBSCAN(eps=0.055, min_samples=1).fit(points[:, [0, 2]])
        labels = clustering.labels_

        # Moyenne des positions pour chaque cylindre
        cyl_positions = []
        for label in set(labels):
            group = points[labels == label]
            center = group.mean(axis=0)
            cyl_positions.append(center)

        cyl_positions = np.array(cyl_positions)
        # Actualiser la vue matplotlib
        ax.clear()
        ax.set_title("Vue de dessus")
        ax.set_xlabel("x (m)")
        ax.set_ylabel("z (m)")
        ax.set_xlim(-dim/2, dim/2)
        ax.set_ylim(0, dim)
        ax.grid(True)
        
        # Cylindres visibles
        for pos in cyl_positions:
            ax.add_patch(plt.Circle((pos[0]*1000, pos[2]*1000), 73/2, color='blue', alpha=0.5))

        if len(cyl_positions) >= 2:
            # Trier selon x
            cyl_positions = cyl_positions[np.argsort(cyl_positions[:, 0])]

            # Centre de la ligne des cylindres
            center_line = cyl_positions.mean(axis=0)

            # Orientation (angle)
            dir_vector = cyl_positions[-1] - cyl_positions[0]
            angle_to_line = np.arctan2(dir_vector[0], dir_vector[2])

            # Affichage console
            print("\n--- Position du robot par rapport au centre de la ligne ---")
            print(f"Position du centre : x = {center_line[0]*1000:.2f} mm, z = {center_line[2]*1000:.2f} mm")
            print(f"Angle robot ↔ ligne : {np.degrees(angle_to_line):.1f}°")

            # Affichage image
            pos = f"x: {center_line[0]*1000:.0f}, z: {center_line[2]*1000:.0f} "
            angle = f"angle: {np.degrees(angle_to_line):.1f}"
            cv2.putText(frame, pos, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(frame, angle, (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Centre ligne
            ax.plot(center_line[0]*1000, center_line[2]*1000, 'gx', markersize=10, label='Centre ligne')

            # Direction (flèche entre les extrêmes)
            ax.arrow(cyl_positions[0][0]*1000, cyl_positions[0][2]*1000,
                    dir_vector[0]*1000, dir_vector[2]*1000,
                    head_width=0.01*1000, head_length=0.02*1000, fc='green', ec='green', label='Orientation')

        ax.plot(0, 0, 'ro', label='Camera')
        ax.legend()
        plt.pause(0.001)

        nb = f"Visible:{len(cyl_positions)}"
        cv2.putText(frame, nb, (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
    # Affichage image
    cv2.imshow("ArUco Positioning", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
