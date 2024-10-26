import numpy as np
import time

class TelloPoseEstimator:
    def __init__(self):
        # State vector [x, y, z, vx, vy, vz, pitch, yaw, roll]
        self.state = np.zeros(9)

        # Process noise covariance matrix (Q)
        self.Q = np.eye(9) * 0.01
        
        # Measurement noise covariance matrix (R)
        self.R = np.eye(9) * 0.1

        # Measurement matrix (H)
        self.H = np.eye(9)  # We directly observe position, velocity, and angles

        # Error covariance matrix (P)
        self.P = np.eye(9)

        # Time of the last update
        self.last_time = time.time()

    def kalman_predict(self, dt):
        # State transition matrix (A)
        A = np.eye(9)
        A[0, 3] = dt  # x depends on vx
        A[1, 4] = dt  # y depends on vy
        A[2, 5] = dt  # z depends on vz

        # Prediction step
        self.state = np.dot(A, self.state)
        self.P = np.dot(np.dot(A, self.P), A.T) + self.Q

    def kalman_update(self, measurement):
        # Update step
        y = measurement - np.dot(self.H, self.state)
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.state = self.state + np.dot(K, y)
        self.P = np.dot(np.eye(9) - np.dot(K, self.H), self.P)

    def update_tello(self, pitch, yaw, roll, vx, vy, vz):
        # Get the current time and calculate time delta (dt)
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Predict step using time delta
        self.kalman_predict(dt)

        # Update the state based on Tello telemetry (pitch, yaw, roll, velocity)
        measurement = np.array([self.state[0], self.state[1], self.state[2], 
                                vx, vy, vz, pitch, yaw, roll])
        self.kalman_update(measurement)

    def vision_update(self, x, y, z, pitch, yaw, roll):
        # Ensure the inputs are scalars
        x = float(x)
        y = float(y)
        z = float(z)
        pitch = float(pitch)
        yaw = float(yaw)
        roll = float(roll)

        # Get the current time and calculate time delta (dt)
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Predict step using time delta
        self.kalman_predict(dt)

        # Update based on vision system (AprilTag pose)
        measurement = np.array([x, y, z, self.state[3], self.state[4], self.state[5], 
                                pitch, yaw, roll])
        self.kalman_update(measurement)


    def get_state(self):
        # Return the current estimated state
        return {
            'x': self.state[0],
            'y': self.state[1],
            'z': self.state[2],
            'vx': self.state[3],
            'vy': self.state[4],
            'vz': self.state[5],
            'pitch': self.state[6],
            'yaw': self.state[7],
            'roll': self.state[8]
        }