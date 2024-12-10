import cv2
import numpy as np

# Define the dimensions of the chessboard
chessboard_size = (8, 6)  # Number of inner corners per chessboard row and column
square_size = 0.0235  # Size of a square in meters (23.5mm)

# Prepare object points
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2) * square_size

# Arrays to store object points and image points from all the images
objpoints = []  # 3D points in real-world space
imgpoints = []  # 2D points in image plane

# Open camera
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

print("Press 'c' to capture images for calibration and 'q' to quit.")

while True:
    ret, frame = cap.read()

    if not ret:
        print("Error: Failed to capture image.")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    # If found, add object points and image points
    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)

        # Draw and display the corners
        cv2.drawChessboardCorners(frame, chessboard_size, corners, ret)

    cv2.imshow('Chessboard Detection', frame)

    key = cv2.waitKey(1)
    if key == ord('c'):  # Capture image
        if ret:
            print("Captured image for calibration.")
        else:
            print("Chessboard not found, capture failed.")
    elif key == ord('q'):  # Quit
        break
# Release the capture
cap.release()
cv2.destroyAllWindows()

# Perform camera calibration
if len(objpoints) > 0:
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    print("Camera matrix:")
    print(camera_matrix)
    print("Distortion coefficients:")
    print(dist_coeffs)
else:
    print("No images captured for calibration.")
