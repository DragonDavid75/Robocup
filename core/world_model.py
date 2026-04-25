# core/world_model.py
import threading
import numpy as np

class WorldModel:
    def __init__(self):
        self.lock = threading.Lock()

        # --- Time stamps --- Always update these from the main threads and respective threads
        self.last_update_time_motion = 0.0
        self.last_update_time_sensors = 0.0
        self.last_update_time_manager = 0.0

        # --- Robot state ---
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_h = 0.0
        self.velocity = 0.0
        self.turnrate = 0.0
        self.imu_heading = 0.0
        self.imu_gyro_z = 0.0

        # --- Sensors ---
        self.line_detected = False
        self.line_left = 0.0
        self.line_right = 0.0
        self.ir_values = []
        self.image = None

        # --- Other variables ---
        self.gripper_distance = 0.24
        self.stage_2 = 0.2

        # --- Mission state ---
        self.current_task = None
        self.mission_state = "IDLE"

    def set_pose(self, x, y, h):
        with self.lock:
            self.pose_x = x
            self.pose_y = y
            self.pose_h = h

    def set_imu(self, heading, gyro_z):
        with self.lock:
            self.imu_heading = heading
            self.imu_gyro_z = gyro_z

    def get_motion(self):
        with self.lock:
            return self.desired_velocity, self.desired_turnrate

    def get_pose(self):
        with self.lock:
            return self.pose_x, self.pose_y, self.pose_h

    def get_imu(self):
        with self.lock:
            return self.imu_heading, self.imu_gyro_z

    def get_velocity(self):
        with self.lock:
            return self.velocity
