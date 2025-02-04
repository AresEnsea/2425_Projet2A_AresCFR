import cv2
import numpy as np
import cv2.aruco as aruco
import matplotlib.pyplot as plt

# Camera matrix and distortion coefficients
camera_matrix = np.array([[520.93177019, 0, 325.9688063], [0, 520.36977644, 215.83389937], [0, 0, 1]])
dist_coeffs = np.array([0.1239, -0.9694, -0.0067, 0.0011, 2.0996])

# Fixed ArUco marker positions in world coordinates (will be initialized later)
fixed_arucos = {
    20: None,
    22: None,
    21: None,
    23: None
}

# ArUco marker for robot
robot_aruco_id = 32
ARUCO_SIZE = 0.02  # Marker size (meters)

# 3D coordinates of marker corners
object_points = np.array([
    [-ARUCO_SIZE / 2, ARUCO_SIZE / 2, 0],
    [ARUCO_SIZE / 2, ARUCO_SIZE / 2, 0],
    [ARUCO_SIZE / 2, -ARUCO_SIZE / 2, 0],
    [-ARUCO_SIZE / 2, -ARUCO_SIZE / 2, 0]
], dtype=np.float32)

# Initialize real-time plot
plt.ion()
fig, ax = plt.subplots()
robot_xs, robot_ys = [], []
plot, = ax.plot(robot_xs, robot_ys, 'bo-', label="Robot Path")
ax.set_xlim(-0.5, 2.0)
ax.set_ylim(-0.5, 2.0)
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.legend()

# Global variables to store camera pose and rotation matrix
camera_tvec, camera_rvec = None, None
R_camera_world = None

def transform_to_world(tvec_camera, R_camera_world, T_camera_world):
    """
    Transform a point from the camera frame to the world frame.
    """
    tvec_camera = np.array(tvec_camera).reshape(3, 1)
    T_camera_world = np.array(T_camera_world).reshape(3, 1)
    tvec_world = np.dot(R_camera_world.T, tvec_camera - T_camera_world)
    return tvec_world.ravel()

def initialize_fixed_arucos(frame):
    """
    Detect and memorize positions of fixed ArUco markers.
    """
    global camera_tvec, camera_rvec, R_camera_world

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)
        world_points, image_points = [], []

        for i, marker_id in enumerate(ids.flatten()):
            if marker_id in fixed_arucos:
                world_points.append(fixed_arucos[marker_id])
                image_points.append(corners[i][0])

        if len(world_points) >= 4:
            world_points = np.array(world_points, dtype=np.float32)
            image_points = np.array([pts[0] for pts in image_points], dtype=np.float32)
            _, camera_rvec, camera_tvec = cv2.solvePnP(world_points, image_points, camera_matrix, dist_coeffs)
            R_camera_world, _ = cv2.Rodrigues(camera_rvec)

            for i, marker_id in enumerate(ids.flatten()):
                if marker_id in fixed_arucos:
                    fixed_arucos[marker_id] = transform_to_world(corners[i][0], R_camera_world, camera_tvec)

            print("Fixed ArUco markers initialized:")
            for marker_id, position in fixed_arucos.items():
                print(f"ID {marker_id}: Position {position}")
        else:
            print("Error: Not enough ArUco markers detected for initialization.")
    else:
        print("Error: No ArUco markers detected.")

def track_robot(frame):
    """
    Track the robot's position using its ArUco marker.
    """
    global camera_tvec, camera_rvec, R_camera_world

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)

        for i, marker_id in enumerate(ids.flatten()):
            if marker_id == robot_aruco_id:
                marker_corners = corners[i][0]
                _, robot_rvec, robot_tvec = cv2.solvePnP(object_points, marker_corners, camera_matrix, dist_coeffs)
                robot_position_camera = robot_tvec.ravel()
                robot_position_world = transform_to_world(robot_position_camera, R_camera_world, camera_tvec)

                # Plot and display robot position
                robot_x, robot_y, robot_z = robot_position_world
                print(f"Robot Position: X={robot_x:.2f}, Y={robot_y:.2f}, Z={robot_z:.2f}")
                robot_xs.append(robot_x)
                robot_ys.append(robot_y)
                plot.set_data(robot_xs, robot_ys)
                plt.draw()
                plt.pause(0.01)
                break

def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    # Initialize fixed ArUco markers
    print("Initializing fixed ArUco markers...")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture frame.")
            break
        initialize_fixed_arucos(frame)
        if all(pos is not None for pos in fixed_arucos.values()):
            break

    print("Fixed ArUco markers successfully initialized.")

    # Start tracking the robot
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture frame.")
            break

        track_robot(frame)
        cv2.imshow("Robot Tracking", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
 