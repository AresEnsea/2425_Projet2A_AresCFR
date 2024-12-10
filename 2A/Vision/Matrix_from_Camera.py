import cv2
import numpy as np
import cv2.aruco as aruco

def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
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

# Définition des paramètres
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
aruco_params = aruco.DetectorParameters()
camera_matrix = np.array([[520.93177019, 0, 325.9688063],[0, 520.36977644, 215.83389937],[0, 0, 1]])  # Matrice intrinsèque
dist_coeffs = np.array([1.23927919e-01, -9.69487805e-01, -6.72723508e-03,  1.06944573e-03, 2.09962148e+00]) # Coefficients de distorsion
table_size = (300, 200)  # Dimensions de la matrice (en pixels)
aruco_fixed_coords = [
    (0.6, 0.6), (0.6, 1.4), (2.4, 0.6), (2.4, 1.4)
]  # Coordonnées des ArUcos fixes en mètres
aruco_fixed_size = (12, 12)  # Dimensions des ArUcos fixes en cm
aruco_mobile_size = (2, 2)  # Taille de la représentation du 5e ArUco dans la matrice

# Calcul des dimensions en pixels
table_pixel_size = (3.0 * 100, 2.0 * 100)  # Dimensions de la table en cm (300x200)
scale_x = table_size[0] / table_pixel_size[0]
scale_y = table_size[1] / table_pixel_size[1]

# Initialisation de la matrice
table_matrix = np.zeros(table_size, dtype=int)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)


# Définir les bordures comme inaccessibles
table_matrix[0, :] = 1
table_matrix[-1, :] = 1
table_matrix[:, 0] = 1
table_matrix[:, -1] = 1

# Fonction pour convertir les coordonnées du monde en indices de matrice
def world_to_matrix(x, y):
    i = int((3.0 - x) * scale_x)
    j = int((2.0 - y) * scale_y)
    return i, j

# Marquer les ArUcos fixes dans la matrice
for coord in aruco_fixed_coords:
    i, j = world_to_matrix(*coord)
    table_matrix[i:i + int(aruco_fixed_size[0] * scale_x),
                 j:j + int(aruco_fixed_size[1] * scale_y)] = 1

# Détection d'ArUcos dans le flux vidéo
cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Détection des ArUcos
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    detector = aruco.ArucoDetector(aruco_dict, parameters)
    if ids is not None:
        for corner, aruco_id in zip(corners, ids.flatten()):
            # Détection du 5e ArUco (mobile)
            if aruco_id == 5:
                rvec, tvec, _ = my_estimatePoseSingleMarkers(corner, 0.12, camera_matrix, dist_coeffs)
                # Correctly unpack the translation vector
                x, y, _ = tvec[0].flatten()  # Now unpacking into three variables # Coordonnées du 5e ArUco en mètres
                i, j = world_to_matrix(x, y)

                # Mise à jour de la matrice
                table_matrix[table_matrix == 2] = 0  # Réinitialiser la position de l'ArUco     mobile
                table_matrix[i:i + aruco_mobile_size[0],
                             j:j + aruco_mobile_size[1]] = 2
                # Sauvegarde de la matrice dans un fichier texte
                with open("C:/Users/zouha/Desktop/table_matrix.txt", "w") as f:
                    np.savetxt(f, table_matrix, fmt='%d')

    # Affichage du flux vidéo avec les ArUcos détectés
    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    cv2.imshow("Frame", frame)
    print(table_matrix)

    # Quitter avec la touche 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()