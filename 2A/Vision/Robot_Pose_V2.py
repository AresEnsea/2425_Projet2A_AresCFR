import cv2
import numpy as np
import cv2.aruco as aruco
import matplotlib.pyplot as plt
import time
from collections import deque

# Camera calibration parameters
camera_matrix1 = np.array([[520.93177019, 0, 325.9688063], [0, 520.36977644, 215.83389937], [0, 0, 1]])
dist_coeffs1 = np.array([0.1239, -0.9694, -0.0067, 0.0011, 2.0996])

camera_matrix2 = np.array([[520.93177019, 0, 325.9688063], [0, 520.36977644, 215.83389937], [0, 0, 1]])
dist_coeffs2 = np.array([0.1239, -0.9694, -0.0067, 0.0011, 2.0996])

# Fixed ArUco marker IDs
fixed_arucos = {20: None, 21: None, 22: None, 23: None}
robot_aruco_id = 32
ARUCO_SIZE = 0.041  # Marker size in meters

# Stockage des positions du robot et des marqueurs
trajectory = deque(maxlen=100)
last_detected_positions = {}

def convert_to_cm(position):
    return position * 225

def main():
    cap1 = cv2.VideoCapture(0)
    cap2 = cv2.VideoCapture(2)

    if not cap1.isOpened() or not cap2.isOpened():
        print("Error: Could not open one or both cameras.")
        return

    plt.ion()
    fig, ax = plt.subplots()

    while True:
        start_time = time.time()

        ret1, frame1 = cap1.read()
        ret2, frame2 = cap2.read()
        if not ret1 or not ret2:
            print("Error: Failed to capture frame from one or both cameras.")
            break

        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        parameters = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(aruco_dict, parameters)

        gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
        corners1, ids1, _ = detector.detectMarkers(gray1)
        corners2, ids2, _ = detector.detectMarkers(gray2)

        positions1, positions2 = {}, {}

        if ids1 is not None:
            for i, marker_id in enumerate(ids1.flatten()):
                _, rvec, tvec = cv2.solvePnP(
                    np.array([[-ARUCO_SIZE / 2, ARUCO_SIZE / 2, 0],
                              [ARUCO_SIZE / 2, ARUCO_SIZE / 2, 0],
                              [ARUCO_SIZE / 2, -ARUCO_SIZE / 2, 0],
                              [-ARUCO_SIZE / 2, -ARUCO_SIZE / 2, 0]], dtype=np.float32),
                    corners1[i][0],
                    camera_matrix1, dist_coeffs1
                )
                positions1[marker_id] = (tvec, rvec)

        if ids2 is not None:
            for i, marker_id in enumerate(ids2.flatten()):
                _, rvec, tvec = cv2.solvePnP(
                    np.array([[-ARUCO_SIZE / 2, ARUCO_SIZE / 2, 0],
                              [ARUCO_SIZE / 2, ARUCO_SIZE / 2, 0],
                              [ARUCO_SIZE / 2, -ARUCO_SIZE / 2, 0],
                              [-ARUCO_SIZE / 2, -ARUCO_SIZE / 2, 0]], dtype=np.float32),
                    corners2[i][0],
                    camera_matrix2, dist_coeffs2
                )
                positions2[marker_id] = (tvec, rvec)

        # Détection des marqueurs fixes
        detected_arucos = {}
        for marker_id in fixed_arucos.keys():
            if marker_id in positions1 and marker_id in positions2:
                tvec = (positions1[marker_id][0] + positions2[marker_id][0]) / 2
            elif marker_id in positions1:
                tvec = positions1[marker_id][0]
            elif marker_id in positions2:
                tvec = positions2[marker_id][0]
            else:
                continue

            x_cm, y_cm = convert_to_cm(tvec[0][0]), -convert_to_cm(tvec[1][0])
            detected_arucos[marker_id] = (x_cm, y_cm)

        # Détection du robot avec gestion explicite des non-détections
        robot_position = None
        if robot_aruco_id in positions1 or robot_aruco_id in positions2:
            if robot_aruco_id in positions1 and robot_aruco_id in positions2:
                tvec = (positions1[robot_aruco_id][0] + positions2[robot_aruco_id][0]) / 2
                rvec = (positions1[robot_aruco_id][1] + positions2[robot_aruco_id][1]) / 2
            elif robot_aruco_id in positions1:
                tvec, rvec = positions1[robot_aruco_id]
            else:
                tvec, rvec = positions2[robot_aruco_id]

            if tvec is not None:
                x_cm, y_cm = convert_to_cm(tvec[0][0]), -convert_to_cm(tvec[1][0])
                robot_position = (x_cm, y_cm)
                trajectory.append((time.time(), x_cm, y_cm))
        else:
            # Si le robot n'est pas détecté, on ne garde pas d'ancienne position
            if robot_aruco_id in last_detected_positions:
                del last_detected_positions[robot_aruco_id]

        # Nettoyage de la trajectoire (points trop anciens)
        current_time = time.time()
        while trajectory and trajectory[0][0] < current_time - 3:
            trajectory.popleft()

        # Mise à jour de l'affichage
        ax.clear()
        ax.set_xlim(-225, 25)
        ax.set_ylim(-100, 20)
        ax.grid(True, linestyle='--', linewidth=0.5, alpha=0.7)

        # Calcul de l'offset basé sur le marqueur 22
        if 22 in detected_arucos:
            x_22, y_22 = detected_arucos[22]
        else:
            x_22, y_22 = 0, 0

        # Affichage des marqueurs fixes
        for marker_id, (x_cm, y_cm) in detected_arucos.items():
            ax.scatter(x_cm - x_22, y_cm - y_22, c='b', label=f'ArUco {marker_id}')
            ax.text(x_cm - x_22, y_cm - y_22 + 1, str(marker_id), fontsize=10, color='blue', ha='center')

        # Affichage du robot uniquement si détecté
        if robot_position is not None:
            ax.scatter(robot_position[0] - x_22, robot_position[1] - y_22, c='r', label='Robot')

        # Affichage de la trajectoire
        if len(trajectory) > 1:
            _, x_vals, y_vals = zip(*trajectory)
            ax.plot([x - x_22 for x in x_vals], [y - y_22 for y in y_vals], c='g', linestyle='-', linewidth=2, alpha=0.7, label="Trajectoire")

        ax.legend()
        plt.pause(0.05)

        cv2.imshow("Camera 1", frame1)
        cv2.imshow("Camera 2", frame2)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap1.release()
    cap2.release()
    cv2.destroyAllWindows()
    plt.ioff()
    plt.show()

if __name__ == "__main__":
    main()
