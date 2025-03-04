import cv2
import numpy as np
from pyzbar.pyzbar import decode
import cv2.aruco as aruco

qr_code_width = 100  # millimeters
map_width = 3000
map_height = 2000

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
        print("Detected ArUco markers")
        aruco.drawDetectedMarkers(frame, corners, ids)
        for i in range(len(ids)):
            c = corners[i][0]
            x, y = int(c[0][0]), int(c[0][1])
            cv2.putText(frame, str(ids[i][0]), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            print(f"ID: {ids[i][0]}")

    return frame


def main():
    # Open two cameras
    cap1 = cv2.VideoCapture(0)
    cap2 = cv2.VideoCapture(1)

    if not cap1.isOpened() or not cap2.isOpened():
        print("Error: Could not open one or both cameras.")
        return

    while True:
        # Read frames from both cameras
        ret1, frame1 = cap1.read()
        ret2, frame2 = cap2.read()

        if not ret1 or not ret2:
            print("Error: Failed to capture images from one or both cameras.")
            break

        # Process the frames
        frame1, qr_data_dict1 = draw_qr_codes(frame1)
        frame1 = detect_aruco(frame1)

        frame2, qr_data_dict2 = draw_qr_codes(frame2)
        frame2 = detect_aruco(frame2)

        # Display frames in separate windows
        cv2.imshow('Camera 1 - QR Code and ArUco Detector', frame1)
        cv2.imshow('Camera 2 - QR Code and ArUco Detector', frame2)

        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release cameras and close windows
    cap1.release()
    cap2.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
