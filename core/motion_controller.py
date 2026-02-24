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

    def run(self):
        while self.running:
            if self.current_task == 'line':
                self._handle_line_mission()
            elif self.current_task == 'turn':
                self._handle_turn_mission()
            else:
                # No active mission, stop the robot to be safe
                self.robot.stop()
            
            time.sleep(0.05)  # 20 Hz

    def _handle_line_mission(self):
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
            # The actual PID control happens inside sedge.py/RobotInterface
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
            turn_speed = 0.8 if error > 0 else -0.8
            self.robot.set_velocity(0, turn_speed)

    # --- High Level Commands for Mission Logic ---

    def follow_until_intersection(self, velocity):
        """Follows the line and stops automatically at a cross-line."""
        print(f"% MotionController: Following line at {velocity} m/s")
        self.robot.set_line_control(velocity, True)
        self.current_task = 'line'

    def turn_in_place(self, angle_rad):
        """Turns the robot by a specific amount of radians."""
        self.robot.set_line_control(0, False) # Ensure line follow is off
        current_h = self.robot.get_pose()["h"]
        self.task_params["target_angle"] = current_h + angle_rad
        self.current_task = 'turn'

    def turn_around(self):
        """180 degree turn."""
        import math
        self.turn_in_place(math.pi)

    def stop(self):
        self.running = False
        self.robot.stop()