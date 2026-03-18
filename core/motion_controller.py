# core/motion_controller.py
import threading
import time
import math
from mqtt_python.uservice import service
from mqtt_python.sedge import edge

class MotionController(threading.Thread):
    def __init__(self, world, robot):
        super().__init__()
        self.world = world
        self.robot = robot
        self.running = True
        self.current_task = None # 'line', 'turn', or None
        self.task_params = {}
        self.distance_ratio = 1.0 # Distance ratio to drive (calibrated experimentally)

    def run(self):
        while self.running:
            if self.current_task == 'line_intersection_or_end':
                self._handle_line_intersection_or_end_mission()
            elif self.current_task == 'line_distance':
                self._handle_line_distance_mission()
            elif self.current_task == 'turn':
                self._handle_turn_mission()
            elif self.current_task == 'drive_distance':
                self._handle_drive_distance_mission()
            elif self.current_task == 'servo_control':
                self._handle_servo_control()
            elif service.stop:
                print("% MotionController: Emergency stop activated!")
                self.stop()
            else:
                # No active mission, stop the robot to be safe
                self.robot.stop()
            
            time.sleep(0.05)  # 20 Hz

    def _handle_line_intersection_or_end_mission(self):
        """Logic for following line until an intersection or end of line."""
        data = self.robot.get_line_data()

        # print(f"% MotionController: Line data - crossingLine: {data['crossingLine']}, lineValidCnt: {data['lineValidCnt']}")

        if edge.crossingLine or edge.lineValidCnt == 0:
            print("% MotionController: Intersection or end of line detected!")
            self.robot.stop()
            self.current_task = None
        else:
            # Maintain the line follow command
            pass

    def _handle_line_distance_mission(self):
        """Logic for following line for a specific distance."""
        current_pos = self.world.get_pose()
        prev_pos = self.task_params.get("prev_pos", 0)
        dist_traveled = self.task_params.get("dist_traveled", 0)
        # get distance just with x,y coordinates
        dist_traveled += math.sqrt((current_pos[0] - prev_pos[0])**2 + (current_pos[1] - prev_pos[1])**2)
        target_distance = self.task_params.get("target_distance", 0)

        self.task_params["dist_traveled"] = dist_traveled
        self.task_params["prev_pos"] = current_pos

        print(f"% MotionController: Driving distance - current position: ({current_pos}), start position: ({prev_pos}), traveled: {dist_traveled:.2f} m, target: {target_distance:.2f} m")

        if dist_traveled >= target_distance:
            print("% MotionController: Line follow distance complete.")
            self.robot.stop()
            self.current_task = None
        else:
            # Maintain the drive command
            pass

    def _handle_drive_distance_mission(self):
        """Logic for driving a specific distance."""
        current_pos = self.world.get_pose()
        prev_pos = self.task_params.get("prev_pos", 0)
        dist_traveled = self.task_params.get("dist_traveled", 0)
        # get distance just with x,y coordinates
        dist_traveled += math.sqrt((current_pos[0] - prev_pos[0])**2 + (current_pos[1] - prev_pos[1])**2)
        target_distance = self.task_params.get("target_distance", 0)

        self.task_params["dist_traveled"] = dist_traveled
        self.task_params["prev_pos"] = current_pos

        print(f"% MotionController: Driving distance - current position: ({current_pos}), start position: ({prev_pos}), traveled: {dist_traveled:.2f} m, target: {target_distance:.2f} m")

        if dist_traveled >= target_distance:
            print("% MotionController: Drive distance complete.")
            self.robot.stop()
            self.current_task = None
        else:
            # Maintain the drive command
            pass

    def _handle_turn_mission(self):
        """Logic for turning in place to a relative heading."""
        target_angle = self.task_params.get("target_angle", 0)
        current_angle = self.world.get_imu()[0]
        error = target_angle - current_angle

        print(f"% MotionController: Turning - current angle: {current_angle:.2f} rad, target angle: {target_angle:.2f} rad, error: {error:.2f} rad")
        
        if abs(error) < 0.03: # Radians tolerance
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
        self.current_task = None # Assume servo command is instantaneous for simplicity

    # --- High Level Commands for Mission Logic ---

    def follow_until_intersection_or_end_line(self, velocity, left_side=True, ref_pos=0.0):
        """Follows the line and stops automatically at a cross-line."""
        print(f"% MotionController: Following line at {velocity} m/s")
        self.robot.reset_trip()
        self.robot.set_line_control(velocity, left_side, ref_pos)
        self.current_task = 'line_intersection_or_end'

    def follow_for_distance(self, distance, velocity, left_side=True, ref_pos=0.0):
        """Follows the line for a specific distance, then stops."""
        print(f"% MotionController: Following line for {distance} meters at {velocity} m/s")
        self.task_params["prev_pos"] = self.world.get_pose()
        self.task_params["dist_traveled"] = 0
        self.task_params["target_distance"] = distance * self.distance_ratio
        self.task_params["velocity"] = velocity
        self.current_task = 'line_distance'
        self.robot.set_line_control(velocity, left_side, ref_pos)

    def drive_distance(self, distance, velocity):
        """Drives a specific distance in meters."""
        print(f"% MotionController: Driving {distance} meters at {velocity} m/s")
        self.robot.set_line_control(0, False) # Ensure line follow is off
        self.task_params["prev_pos"] = self.world.get_pose()
        self.task_params["dist_traveled"] = 0
        self.task_params["target_distance"] = distance * self.distance_ratio
        self.task_params["velocity"] = velocity
        self.current_task = 'drive_distance'

        self.robot.set_velocity(velocity, 0)

    def turn_in_place(self, angle_rad):
        """Turns the robot by a specific amount of radians."""
        self.robot.set_line_control(0, False) # Ensure line follow is off
        current_h = self.world.get_imu()[0]
        self.task_params["target_angle"] = current_h + angle_rad
        self.current_task = 'turn'

        # Positive angle = turn left (positive angular velocity)
        # Negative angle = turn right (negative angular velocity)
        turn_speed = 0.5
        self.robot.set_velocity(0.3, turn_speed)

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