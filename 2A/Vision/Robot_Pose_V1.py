import cv2
import numpy as np
import cv2.aruco as aruco
import matplotlib.pyplot as plt

# Camera matrix and distortion coefficients
camera_matrix = np.array([[520.93177019, 0, 325.9688063], [0, 520.36977644, 215.83389937], [0, 0, 1]])
dist_coeffs = np.array([0.1239, -0.9694, -0.0067, 0.0011, 2.0996])

# Fixed ArUco marker positions in world coordinates
fixed_arucos = {
    20: np.array([0.0, 0.0, 0.0]),
    22: np.array([0.8, 0.0, 0.0]),
    21: np.array([0.0, 1.8, 0.0]),
    23: np.array([0.8, 1.8, 0.0])
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
plot, = ax.plot(10*robot_xs, 10*robot_ys, 'bo-', label="Robot Path")
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.legend()

def transform_to_world(tvec_camera, R_camera_world, T_camera_world):
    """
    Transform a point from the camera frame to the world frame.
    """
    tvec_camera = np.array(tvec_camera).reshape(3, 1)  # Ensure 3x1 array
    T_camera_world = np.array(T_camera_world).reshape(3, 1)  # Ensure 3x1 array
    tvec_world = np.dot(R_camera_world.T, tvec_camera - T_camera_world)
    return tvec_world.ravel()  # Return as 1D array with 3 elements

def detect_aruco(frame):
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)
        print(f"Detected IDs: {ids.flatten()}")  # Debugging: Print detected IDs

        camera_tvec, camera_rvec = None, None
        world_points, image_points = [], []

        for i, marker_id in enumerate(ids.flatten()):
            if marker_id in fixed_arucos:
                world_points.append(fixed_arucos[marker_id])
                image_points.append(corners[i][0])

        if len(world_points) >= 4:  # Ensure enough points for solvePnP
            world_points = np.array(world_points, dtype=np.float32)
            image_points = np.array([pts[0] for pts in image_points], dtype=np.float32)
            _, camera_rvec, camera_tvec = cv2.solvePnP(world_points, image_points, camera_matrix, dist_coeffs)
            print(f"Camera Position (World Frame): {camera_tvec.ravel()}")
        else:
            print("Error: Not enough ArUco markers detected for camera pose estimation.")
            return frame

        if camera_tvec is not None and camera_rvec is not None:
            R_camera_world, _ = cv2.Rodrigues(camera_rvec)
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id == robot_aruco_id:
                    marker_corners = corners[i][0]
                    _, robot_rvec, robot_tvec = cv2.solvePnP(object_points, marker_corners, camera_matrix, dist_coeffs)
                    robot_position_camera = robot_tvec.ravel()
                    print(f"Robot Position Camera: {robot_position_camera}")  # Debugging
                    robot_position_world = transform_to_world(robot_position_camera, R_camera_world, camera_tvec)
                    print(f"Robot Position World: {robot_position_world}")  # Debugging
                    robot_x, robot_y, robot_z = robot_position_world
                    print(f"Robot Position: X={robot_x:.2f}, Y={robot_y:.2f}, Z={robot_z:.2f}")
                    robot_xs.append(robot_x)
                    robot_ys.append(robot_y)
                    plot.set_data(robot_xs, robot_ys)
                    plt.draw()
                    plt.pause(0.01)

    return frame

def main():
    cap = cv2.VideoCapture(1)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture frame.")
            break

        frame = detect_aruco(frame)
        cv2.imshow("ArUco Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
