import cv2
import numpy as np
import cv2.aruco as aruco
from pyzbar.pyzbar import decode
import matplotlib.pyplot as plt

# Define camera matrix and distortion coefficients (replace with your calibrated values)
camera_matrix = np.array([[520.93177019, 0, 325.9688063], [0, 520.36977644, 215.83389937], [0, 0, 1]])  # Replace with your actual camera matrix
dist_coeffs = np.array([1.23927919e-01, -9.69487805e-01, -6.72723508e-03, 1.06944573e-03, 2.09962148e+00])

# ArUco marker size in meters (adjust to your actual marker size)
ARUCO_SIZE = 0.02  # Example: 5 cm

# Define 3D positions of fixed ArUco markers in the world frame (meters)
fixed_arucos = {
    1: np.array([0.0, 0.0, 0.0]),  # ID 1 at origin
    2: np.array([1.0, 0.0, 0.0]),  # ID 2 at (1, 0, 0)
    3: np.array([1.0, 1.0, 0.0]),  # ID 3 at (1, 1, 0)
    4: np.array([0.0, 1.0, 0.0])   # ID 4 at (0, 1, 0)
}

# ArUco marker ID for the robot
robot_aruco_id = 10  # Choose a unique ID for the robot

# Initialize real-time plotting
plt.ion()
fig, ax = plt.subplots()
robot_xs, robot_ys = [], []
plot, = ax.plot(robot_xs, robot_ys, 'bo-', label="Robot Path")
ax.set_xlim(-0.5, 1.5)
ax.set_ylim(-0.5, 1.5)
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.legend()

def transform_to_world(tvec_camera, R_camera_world, T_camera_world):
    """
    Transform a point from the camera frame to the world frame.
    """
    tvec_world = np.dot(R_camera_world.T, tvec_camera - T_camera_world)
    return tvec_world

def detect_aruco(frame):
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
    parameters = aruco.DetectorParameters()

    detector = aruco.ArucoDetector(aruco_dict, parameters)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)
        camera_tvec, camera_rvec = None, None

        # Extract camera pose from fixed ArUcos
        world_points = []
        image_points = []
        for i, marker_id in enumerate(ids.flatten()):
            if marker_id in fixed_arucos:
                world_points.append(fixed_arucos[marker_id])
                image_points.append(corners[i][0])

        if len(world_points) >= 3:
            world_points = np.array(world_points, dtype=np.float32)
            image_points = np.array([pts[0] for pts in image_points], dtype=np.float32)
            _, camera_rvec, camera_tvec = cv2.solvePnP(world_points, image_points, camera_matrix, dist_coeffs)

        if camera_tvec is not None and camera_rvec is not None:
            R_camera_world, _ = cv2.Rodrigues(camera_rvec)

            # Detect robot marker and calculate its global position
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id == robot_aruco_id:
                    marker_corners = corners[i][0]
                    _, robot_rvec, robot_tvec = cv2.solvePnP(object_points, marker_corners, camera_matrix, dist_coeffs)

                    if robot_tvec is not None:
                        robot_position_camera = robot_tvec.ravel()
                        robot_position_world = transform_to_world(robot_position_camera, R_camera_world, camera_tvec)

                        # Robot position in global frame
                        robot_x, robot_y, robot_z = robot_position_world
                        print(f"Robot Global Position: X={robot_x:.2f}, Y={robot_y:.2f}, Z={robot_z:.2f}")

                        # Update the plot
                        robot_xs.append(robot_x)
                        robot_ys.append(robot_y)
                        plot.set_data(robot_xs, robot_ys)
                        ax.relim()
                        ax.autoscale_view()
                        plt.draw()
                        plt.pause(0.01)

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

        frame = detect_aruco(frame)
        cv2.imshow('ArUco Robot Localization', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
