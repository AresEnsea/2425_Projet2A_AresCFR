import numpy as np

class KalmanFilter:
    def __init__(self, dt, u_noise, z_noise, process_noise):
        """
        Kalman Filter for 2D position tracking.
        Args:
            dt (float): Time step between measurements.
            u_noise (float): Measurement uncertainty.
            z_noise (float): Measurement noise (observation noise).
            process_noise (float): Process noise (model uncertainty).
        """
        self.dt = dt

        # State vector: [x, y, vx, vy]
        self.x = np.zeros((4, 1))

        # State transition matrix (A)
        self.A = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        # Control matrix (B)
        self.B = np.array([
            [0.5 * dt**2, 0],
            [0, 0.5 * dt**2],
            [dt, 0],
            [0, dt]
        ])

        # Observation matrix (H)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        # Process noise covariance (Q)
        self.Q = process_noise * np.eye(4)

        # Measurement noise covariance (R)
        self.R = z_noise * np.eye(2)

        # State covariance matrix (P)
        self.P = np.eye(4) * u_noise

    def predict(self, u=np.zeros((2, 1))):
        """
        Predict the next state and update state covariance.
        Args:
            u (numpy array): Control input (acceleration in x and y).
        Returns:
            numpy array: Predicted state.
        """
        self.x = np.dot(self.A, self.x) + np.dot(self.B, u)
        self.P = np.dot(self.A, np.dot(self.P, self.A.T)) + self.Q
        return self.x

    def update(self, z):
        """
        Update the state with a new observation.
        Args:
            z (numpy array): Observed position [x, y].
        Returns:
            numpy array: Updated state.
        """
        y = z - np.dot(self.H, self.x)  # Innovation
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R  # Innovation covariance
        K = np.dot(self.P, np.dot(self.H.T, np.linalg.inv(S)))  # Kalman gain
        self.x = self.x + np.dot(K, y)  # Update state
        self.P = self.P - np.dot(K, np.dot(self.H, self.P))  # Update covariance
        return self.x

# Example usage
if __name__ == "__main__":
    # Initialize Kalman Filter
    dt = 0.1  # Time step (100ms)
    kalman = KalmanFilter(dt=dt, u_noise=1.0, z_noise=1.0, process_noise=0.01)

    # Simulated measurements (noisy positions)
    measurements = [
        [0, 0],
        [0.1, 0.2],
        [0.4, 0.6],
        [0.9, 1.1],
        [1.6, 1.7],
        [2.5, 2.8]
    ]

    # Apply Kalman filter
    for z in measurements:
        z = np.array(z).reshape(2, 1)
        predicted = kalman.predict()
        updated = kalman.update(z)
        print(f"Measured: {z.ravel()}, Predicted: {predicted[:2].ravel()}, Updated: {updated[:2].ravel()}")
