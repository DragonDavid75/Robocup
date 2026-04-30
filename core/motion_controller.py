import threading
import time
import math
from mqtt_python.uservice import service
from mqtt_python.sedge import edge
import typing

class MotionController(threading.Thread):

    def __init__(self, world, robot, line_follower, ball_task=None):
        super().__init__()
        self.world = world
        self.robot = robot
        self.line_follower = line_follower
        self.ball_task = ball_task

        self.running = True
        self.current_task = None
        self.task_params = {}
        self.distance_ratio = 1.0

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

            elif self.current_task == 'drive_circle':
                self._handle_drive_circle_mission()

            elif self.current_task == 'drive_arc':
                self._handle_drive_arc_mission()

            elif self.current_task == 'drive_to_line':
                self._handle_drive_to_line_mission()

            elif self.current_task == 'drive_to_ball':
                self._handle_drive_to_ball_mission()
            elif self.current_task == 'line_until_loss':
                self._handle_line_until_loss_mission()
            elif service.stop:
                print("% MotionController: Emergency stop activated!")
                self.stop()

            else:
                self.robot.stop()

            time.sleep(0.05)

    def _normalize_angle(self, angle):
        """Wrap angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    # --- Monitoring Logic ---

    def _handle_line_intersection_or_end_mission(self):
        if edge.crossingLine:
            print("% MotionController: Intersection detected!")
            self.line_follower.stop_following()
            self.robot.stop()
            self.current_task = None

    def _handle_line_distance_mission(self):
        current_pos = self.world.get_pose()
        prev_pos = self.task_params.get("prev_pos", current_pos)
        dist_traveled = self.task_params.get("dist_traveled", 0.0)

        dist_traveled += math.sqrt(
            (current_pos[0] - prev_pos[0]) ** 2 +
            (current_pos[1] - prev_pos[1]) ** 2
        )

        self.task_params["dist_traveled"] = dist_traveled
        self.task_params["prev_pos"] = current_pos

        if dist_traveled >= self.task_params.get("target_distance", 0.0):
            print("% MotionController: Line distance complete.")
            self.line_follower.stop_following()
            self.robot.stop()
            self.current_task = None

    def _handle_drive_distance_mission(self):
        current_pos = self.world.get_pose()
        prev_pos = self.task_params.get("prev_pos", current_pos)
        dist_traveled = self.task_params.get("dist_traveled", 0.0)

        dist_traveled += math.sqrt(
            (current_pos[0] - prev_pos[0]) ** 2 +
            (current_pos[1] - prev_pos[1]) ** 2
        )

        self.task_params["dist_traveled"] = dist_traveled
        self.task_params["prev_pos"] = current_pos

        if dist_traveled >= self.task_params.get("target_distance", 0.0):
            print("% MotionController: Drive distance complete.")
            self.robot.stop()
            self.current_task = None

    def _handle_turn_mission(self):
        target_angle = self.task_params.get("target_angle", 0.0)
        current_angle = self.world.get_imu()[0]
        error = self._normalize_angle(target_angle - current_angle)

        if abs(error) < 0.03:
            print("% MotionController: Turn complete.")
            self.robot.stop()
            self.current_task = None

    def _handle_drive_circle_mission(self):
        start_time = self.task_params.get("start_time", 0.0)
        duration = self.task_params.get("duration", 0.0)

        if time.time() - start_time >= duration:
            print("% MotionController: Circle drive complete.")
            self.robot.stop()
            self.current_task = None
    def _get_pose3(self) -> tuple[float, float, float]:
        pose = self.world.get_pose()
        if not isinstance(pose, tuple) or len(pose) < 3:
            return (0.0, 0.0, 0.0)
        return typing.cast(tuple[float, float, float], pose)
    def _handle_drive_arc_mission(self):
        start_angle = self.task_params.get("start_angle", 0.0)
        target_delta = self.task_params.get("target_delta", 0.0)

        current_angle = self.world.get_imu()[0]
        turned = self._normalize_angle(current_angle - start_angle)
        error = self._normalize_angle(target_delta - turned)

        print(f"% MotionController: Arc drive - turned: {turned:.2f}, target: {target_delta:.2f}, error: {error:.2f}")

        if abs(error) < 0.05:
            print("% MotionController: Arc complete.")
            self.robot.stop()
            self.current_task = None
    def _handle_line_until_loss_mission(self):
        """Monitors line validity and stops after line is lost for too long."""
        current_time = time.time()

        if edge.lineValid or edge.lineValidCnt > 0:
            self.task_params["last_valid_time"] = current_time

        last_valid_time = float(self.task_params.get("last_valid_time", current_time) or current_time)
        loss_timeout = float(self.task_params.get("loss_timeout", 0.3) or 0.3)

        if current_time - last_valid_time > loss_timeout:
            print("% MotionController: Line loss detected.")
            self.line_follower.stop_following()
            self.robot.stop()
            self.current_task = None
    def _handle_drive_to_line_mission(self):
        if edge.lineValid:
            print("% MotionController: Line detected!")
            self.robot.stop()
            self.current_task = None

    def _handle_drive_to_ball_mission(self):
        if self.ball_task.is_complete():
            print("% MotionController: Ball reached!")
            self.robot.stop()
            self.current_task = None
    def follow_until_line_loss(self, velocity, action="STRAIGHT", loss_timeout=0.3):
        """
        Follows the line until the line has been lost for longer than loss_timeout.
        """
        print(f"% MotionController: Following line until loss at {velocity}")
        self.task_params = {
            "last_valid_time": time.time(),
            "loss_timeout": loss_timeout,
        }
        self.robot.reset_trip()
        self.line_follower.start_following(velocity, action)
        self.current_task = 'line_until_loss'
    # --- High Level Commands ---

    """
    Follows the line until it detects an intersection currently only.
    Velocity: The speed at which the robot should follow the line.
    """
    def follow_until_intersection_or_end_line(self, velocity):
        print(f"% MotionController: Following line at {velocity}")
        self.robot.reset_trip()
        self.line_follower.start_following(velocity)
        self.current_task = 'line_intersection_or_end'

    """
    Follows the line for a specific distance.
    Distance: The distance to follow the line (in meters).
    Velocity: The speed at which the robot should follow the line.
    Action: The action to perform at intersections. Options are "STRAIGHT", "LEFT", "RIGHT". Default is "STRAIGHT".
    For now only one action can be given, and it will be executed at all intersections.
    """
    def follow_for_distance(self, distance, velocity, action="STRAIGHT"):
        self.task_params = {}
        self.task_params["prev_pos"] = self.world.get_pose()
        self.task_params["dist_traveled"] = 0.0
        self.task_params["target_distance"] = distance * self.distance_ratio

        self.line_follower.start_following(velocity, action)
        self.current_task = 'line_distance'

    """
    Drives straight for a specific distance.
    Distance: The distance to drive (in meters).
    Velocity: The speed at which the robot should drive.
    """
    def drive_distance(self, distance, velocity):
        self.task_params = {}
        self.task_params["prev_pos"] = self.world.get_pose()
        self.task_params["dist_traveled"] = 0.0
        self.task_params["target_distance"] = distance * self.distance_ratio

        self.robot.set_velocity(velocity, 0.0)
        self.current_task = 'drive_distance'

    """
    Turns in place by a specific angle.
    Angle_rad: The angle to turn (in radians). Positive values turn left, negative values turn right.
    """
    def turn_in_place(self, angle_rad):
        current_h = self.world.get_imu()[0]
        self.task_params = {}
        self.task_params["target_angle"] = self._normalize_angle(current_h + angle_rad)

        turn_speed = 0.5 if angle_rad > 0 else -0.5
        self.robot.set_velocity(0.0, turn_speed)
        self.current_task = 'turn'

    """
    Drives in a circle with a specific radius and linear speed.
    Radius: The radius of the circle (in meters).
    Linear_speed: The linear speed at which the robot should drive (in meters per second).
    """
    def drive_circle(self, radius, linear_speed):
        """
        Drive one full circle using radius and linear speed.
        """
        angular_speed = linear_speed / radius
        circumference = 2 * math.pi * radius
        duration = circumference / linear_speed

        self.task_params = {}
        self.task_params["duration"] = duration
        self.task_params["start_time"] = time.time()

        self.robot.set_velocity(linear_speed, angular_speed)
        self.current_task = "drive_circle"

    """
    Drives an arc by a specific angle, radius, and linear speed.
    Angle_rad: The angle to turn (in radians). Positive values turn left, negative values turn right.
    Radius: The radius of the arc (in meters).
    Linear_speed: The linear speed at which the robot should drive (in meters per second).
    """
    def drive_arc(self, angle_rad, radius=0.35, linear_speed=0.05):
        self.robot.set_line_control(0, False)

        angular_speed = linear_speed / radius
        if angle_rad < 0:
            angular_speed = -angular_speed

        self.task_params = {}
        self.task_params["start_angle"] = self.world.get_imu()[0]
        self.task_params["target_delta"] = angle_rad
        self.task_params["angular_speed"] = angular_speed

        self.current_task = 'drive_arc'
        self.robot.set_velocity(linear_speed, angular_speed)

    """
    Drives straight until it detects a line.
    Velocity: The speed at which the robot should drive.
    """
    def drive_to_line(self, velocity):
        print(f"% MotionController: Driving to line at {velocity}")
        self.robot.set_velocity(velocity, 0.0)
        self.current_task = 'drive_to_line'

    """
    Drives straight until it detects the ball using the vision system.
    """
    def drive_to_ball(self):
        print(f"% MotionController: Driving to ball")
        self.ball_task.start()

    """
    Checks if the motion controller is currently executing a task.
    Returns True if a task is in progress, False otherwise.
    Use it to prevent starting a new task while another one is still running.
    """
    def is_busy(self):
        return self.current_task is not None

    """
    Stops the motion controller and the robot immediately.
    """
    def stop(self):
        self.running = False
        self.robot.stop()