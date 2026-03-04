# core/motion_controller.py
import threading
import time

class MotionController(threading.Thread):
    def __init__(self, world, robot):
        super().__init__()
        self.world = world
        self.robot = robot
        self.running = True
        self.current_task = None # 'line', 'turn', or None
        self.task_params = {}
        self.distance_ratio = 0.75 # Distance ratio to drive (calibrated experimentally)

    def run(self):
        while self.running:
            if self.current_task == 'line_intersection':
                self._handle_line_intersection_mission()
            elif self.current_task == 'line_end':
                self._handle_line_end_mission()
            elif self.current_task == 'turn':
                self._handle_turn_mission()
            elif self.current_task == 'drive_distance':
                self._handle_drive_distance_mission()
            elif self.current_task == 'servo_control':
                self._handle_servo_control()
            elif service.stop:
                print("% MotionController: Emergency stop activated!")
            else:
                # No active mission, stop the robot to be safe
                self.robot.stop()
            
            time.sleep(0.05)  # 20 Hz

    def _handle_line_intersection_mission(self):
        """Logic for following line until an intersection."""
        data = self.robot.get_line_data()
        # crossingLine is determined in sedge.py (average reflectivity > threshold)
        from sedge import edge
        
        if edge.crossingLine:
            print("% MotionController: Intersection detected!")
            self.robot.stop()
            self.current_task = None
        else:
            # Maintain the line follow command
            pass

    def _handle_line_end_mission(self):
        """Logic for following line until it ends (no line detected)."""
        data = self.robot.get_line_data()
        from sedge import edge
        
        if not edge.lineDetected:
            print("% MotionController: End of line detected!")
            self.robot.stop()
            self.current_task = None
        else:
            # Maintain the line follow command
            pass

    def _handle_drive_distance_mission(self):
        """Logic for driving a specific distance."""
        current_dist = self.robot.get_odometry()["dist"]
        target_distance = self.task_params.get("target_distance", 0)
        velocity = self.task_params.get("velocity", 0)

        print(f"% MotionController: Driving distance - current: {current_dist:.2f} m, target: {target_distance:.2f} m")

        if current_dist >= target_distance:
            print("% MotionController: Drive distance complete.")
            self.robot.stop()
            self.current_task = None
        else:
            # Maintain the drive command
            pass

    def _handle_turn_mission(self):
        """Logic for turning in place to a relative heading."""
        target_angle = self.task_params.get("target_angle", 0)
        current_angle = self.robot.get_pose()["h"]
        
        error = target_angle - current_angle
        # Basic P-control for turning
        if abs(error) < 0.05: # Radians tolerance
            self.robot.stop()
            self.current_task = None
            print("% MotionController: Turn complete.")
        else:
            # Maintain the turn command
            pass

    def _handle_servo_control(self):
        """Logic for direct servo control."""
        idx = self.task_params.get("servo_idx", 0)
        pos = self.task_params.get("servo_pos", 0)
        speed = self.task_params.get("servo_speed", 0)
        self.robot.set_servo(idx, pos, speed)

    # --- High Level Commands for Mission Logic ---

    def follow_until_intersection(self, velocity, left_side=True, ref_pos=0.0):
        """Follows the line and stops automatically at a cross-line."""
        print(f"% MotionController: Following line at {velocity} m/s")
        self.robot.set_line_control(velocity, left_side, ref_pos)
        self.current_task = 'line_intersection'

    def follow_until_end_of_line(self, velocity, left_side=True, ref_pos=0.0):
        """Follows the line and stops when it ends (no line detected)."""
        print(f"% MotionController: Following line at {velocity} m/s until end of line")
        self.robot.set_line_control(velocity, left_side, ref_pos)
        self.current_task = 'line_end'

    def drive_distance(self, distance, velocity):
        """Drives a specific distance in meters."""
        print(f"% MotionController: Driving {distance} meters at {velocity} m/s")
        self.robot.set_line_control(0, False) # Ensure line follow is off
        self.robot.reset_trip()
        self.task_params["target_distance"] = distance * self.distance_ratio
        self.task_params["velocity"] = velocity
        self.current_task = 'drive_distance'

        self.robot.set_velocity(velocity, 0)

    def turn_in_place(self, angle_rad):
        """Turns the robot by a specific amount of radians."""
        self.robot.set_line_control(0, False) # Ensure line follow is off
        current_h = self.robot.get_pose()["h"]
        self.task_params["target_angle"] = current_h + angle_rad
        self.current_task = 'turn'

        # Positive angle = turn left (positive angular velocity)
        # Negative angle = turn right (negative angular velocity)
        turn_speed = 0.8 if angle_rad > 0 else -0.8
        self.robot.set_velocity(0, turn_speed)

    def turn_around(self):
        """180 degree turn."""
        import math
        self.turn_in_place(math.pi)

    def servo_control(self, idx, pos, speed):
        """Direct servo control."""
        self.task_params["servo_idx"] = idx
        self.task_params["servo_pos"] = pos
        self.task_params["servo_speed"] = speed
        self.current_task = 'servo_control'

    def is_busy(self):
        """Returns True if the robot is currently executing a motion command."""
        return self.current_task is not None

    def stop(self):
        self.running = False
        self.robot.stop()