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

        # --- Sensors ---
        self.line_detected = False
        self.line_left = 0.0
        self.line_right = 0.0
        self.ir_values = []
        self.image = None

        # --- Commands ---
        self.desired_velocity = 0.0
        self.desired_turnrate = 0.0

        # --- Mission state ---
        self.current_task = None
        self.mission_state = "IDLE"

    def set_motion(self, v, w):
        with self.lock:
            self.desired_velocity = v
            self.desired_turnrate = w

    def get_motion(self):
        with self.lock:
            return self.desired_velocity, self.desired_turnrate
