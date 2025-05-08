#!/usr/bin/env python3
import cv2
import numpy as np
import os

# Paramètres du da#!/usr/bin/env python3mier
nb_corners_x = 9  # coins internes horizontalement
nb_corners_y = 6  # coins internes verticalement
square_size_mm = 17.5  # taille réelle d’un carré en mm (à ajuster)

# Préparation des points 3D de référence (repère monde)
objp = np.zeros((nb_corners_y * nb_corners_x, 3), np.float32)
objp[:, :2] = np.mgrid[0:nb_corners_x, 0:nb_corners_y].T.reshape(-1, 2)
objp *= square_size_mm / 1000.0  # en mètres

# Listes de points pour calibration
objpoints = []  # points 3D
imgpoints = []  # points 2D

cap = cv2.VideoCapture(2)  # modifie l'index si besoin
cv2.namedWindow('Calibration', cv2.WINDOW_NORMAL) 
cv2.resizeWindow('Calibration', 800, 600)

print("Appuie sur 'espace' pour capturer une image, 'q' pour quitter.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret_corners, corners = cv2.findChessboardCorners(gray, (nb_corners_x, nb_corners_y), None)

    if ret_corners:
        cv2.drawChessboardCorners(frame, (nb_corners_x, nb_corners_y), corners, ret_corners)

    cv2.imshow('Calibration', frame)
    key = cv2.waitKey(1)

    if key == ord(' '):
        if ret_corners:
            objpoints.append(objp)
            imgpoints.append(corners)
            print(f"[{len(objpoints)}] Capture enregistrée.")
        else:
            print("❌ Damier non détecté.")

    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

if len(objpoints) >= 10:
    print("📐 Calibration en cours...")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    np.save('camera_matrix.npy', mtx)
    np.save('dist_coeffs.npy', dist)

    print("✅ Calibration réussie ! Fichiers enregistrés :")
    print("- camera_matrix.npy")
    print("- dist_coeffs.npy")
else:
    print("⚠️ Pas assez d'images pour calibrer (au moins 10 bonnes prises nécessaires)")
