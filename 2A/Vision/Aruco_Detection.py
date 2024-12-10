import cv2
import numpy as np
import cv2.aruco as aruco
from pyzbar.pyzbar import decode

# Define camera matrix and distortion coefficients (replace with your calibrated values)
camera_matrix = np.array([[520.93177019, 0, 325.9688063],[0, 520.36977644, 215.83389937],[0, 0, 1]])  # Replace with your actual camera matrix
dist_coeffs = np.array([1.23927919e-01, -9.69487805e-01, -6.72723508e-03,  1.06944573e-03, 2.09962148e+00])

# ArUco marker size in meters (adjust to your actual marker size)
ARUCO_SIZE = 0.02  # Example: 5 cm

qr_code_width = 100  # millimeters
map_width = 3000
map_heigh = 2000

# Define 3D coordinates of the marker's corners (assuming it's a flat square on the XY plane)
object_points = np.array([
    [-ARUCO_SIZE / 2, ARUCO_SIZE / 2, 0],
    [ARUCO_SIZE / 2, ARUCO_SIZE / 2, 0],
    [ARUCO_SIZE / 2, -ARUCO_SIZE / 2, 0],
    [-ARUCO_SIZE / 2, -ARUCO_SIZE / 2, 0]
], dtype=np.float32)

def draw_qr_codes(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    qr_codes = decode(gray)
    qr_data_dict = {}

    for qr_code in qr_codes:
        points = qr_code.polygon
        if len(points) == 4:
            pts = [(point.x, point.y) for point in points]
            pts = np.array(pts, dtype=np.int32)
            cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

            qr_data = qr_code.data.decode('utf-8')
            qr_data_dict[qr_data] = pts.tolist()
            x, y = pts[0][0], pts[0][1]
            cv2.putText(frame, qr_data, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    return frame, qr_data_dict

def detect_aruco(frame):
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
    parameters = aruco.DetectorParameters()

    detector = aruco.ArucoDetector(aruco_dict, parameters)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = detector.detectMarkers(gray)

    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)

        for i in range(len(ids)):
            # Get the corner points for this marker
            marker_corners = corners[i][0]

            # Estimate pose using cv2.solvePnP
            ret, rvec, tvec = cv2.solvePnP(object_points, marker_corners, camera_matrix, dist_coeffs)

            if ret:
                # Draw the 3D axis for each marker
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, ARUCO_SIZE)

                # Print pose information for debugging
                print(f"Marker ID: {ids[i][0]}")
                print(f"Rotation Vector (rvec): {rvec.ravel()}")
                print(f"Translation Vector (tvec): {tvec.ravel()}")

    return frame

def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture image.")
            break

        qr_data_dict = {}
        frame, qr_data_dict = draw_qr_codes(frame)

        frame = detect_aruco(frame)

        cv2.imshow('QR Code and ArUco Detector', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
