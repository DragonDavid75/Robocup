# core/motion_controller.py
import threading
import time
import math
from mqtt_python.uservice import service
from mqtt_python.sedge import edge

class MotionController(threading.Thread):
    def __init__(self, world, robot, line_follower):
        super().__init__()
        self.world = world
        self.robot = robot
        self.line_follower = line_follower
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
            elif service.stop:
                print("% MotionController: Emergency stop activated!")
                self.stop()
            else:
                # No active mission, make sure the robot is safely stopped
                self.robot.stop()
            
            time.sleep(0.05)  # 20 Hz monitoring loop

    # --- Monitoring Logic (Runs in Background Thread) ---

    def _handle_line_intersection_or_end_mission(self):
        """Monitors for an intersection or end of line."""
        if edge.crossingLine:
            print("% MotionController: Intersection or end of Line detected!")
            self.line_follower.stop_following()
            self.robot.stop()
            self.current_task = None

    def _handle_line_distance_mission(self):
        """Monitors distance traveled while line following."""
        current_pos = self.world.get_pose()
        prev_pos = self.task_params.get("prev_pos", 0)
        dist_traveled = self.task_params.get("dist_traveled", 0)
        
        # Update distance with x,y coordinates
        dist_traveled += math.sqrt((current_pos[0] - prev_pos[0])**2 + (current_pos[1] - prev_pos[1])**2)
        self.task_params["dist_traveled"] = dist_traveled
        self.task_params["prev_pos"] = current_pos

        if dist_traveled >= self.task_params.get("target_distance", 0):
            print("% MotionController: Line follow distance complete.")
            self.line_follower.stop_following()
            self.robot.stop()
            self.current_task = None

    def _handle_drive_distance_mission(self):
        """Monitors distance traveled while driving straight."""
        current_pos = self.world.get_pose()
        prev_pos = self.task_params.get("prev_pos", 0)
        dist_traveled = self.task_params.get("dist_traveled", 0)
        
        # Update distance with x,y coordinates
        dist_traveled += math.sqrt((current_pos[0] - prev_pos[0])**2 + (current_pos[1] - prev_pos[1])**2)
        self.task_params["dist_traveled"] = dist_traveled
        self.task_params["prev_pos"] = current_pos

        if dist_traveled >= self.task_params.get("target_distance", 0):
            print("% MotionController: Drive distance complete.")
            self.robot.stop()
            self.current_task = None

    def _handle_turn_mission(self):
        """Monitors the IMU to check if the target angle is reached."""
        target_angle = self.task_params.get("target_angle", 0)
        current_angle = self.world.get_imu()[0]
        error = target_angle - current_angle

        if abs(error) < 0.03: # Radians tolerance
            print("% MotionController: Turn complete.")
            self.robot.stop()
            self.current_task = None

    def _handle_drive_circle_mission(self):
        """Monitors the timer for a circle driving maneuver."""
        start_time = self.task_params.get("start_time", 0)
        duration = self.task_params.get("duration", 0)        

        if time.time() - start_time >= duration:
            print("% MotionController: Circle drive complete.")
            self.robot.stop()
            self.current_task = None

    # --- High Level Commands (Called by Main Thread) ---

    def follow_until_intersection_or_end_line(self, velocity, left_side=True, ref_pos=0.0):
        print(f"% MotionController: Following Line at {velocity} m/s")
        self.robot.reset_trip()
        self.line_follower.start_following(velocity) # Hardware command sent
        self.current_task = 'line_intersection_or_end' # Thread begins monitoring

    def follow_for_distance(self, distance, velocity, left_side=True, ref_pos=0.0):
        print(f"% MotionController: Following line for {distance} meters at {velocity} m/s")
        self.task_params["prev_pos"] = self.world.get_pose()
        self.task_params["dist_traveled"] = 0
        self.task_params["target_distance"] = distance * self.distance_ratio
        
        self.line_follower.start_following(velocity) # Hardware command sent
        self.current_task = 'line_distance' # Thread begins monitoring

    def drive_distance(self, distance, velocity):
        print(f"% MotionController: Driving {distance} meters at {velocity} m/s")
        self.task_params["prev_pos"] = self.world.get_pose()
        self.task_params["dist_traveled"] = 0
        self.task_params["target_distance"] = distance * self.distance_ratio
        
        self.robot.set_velocity(velocity, 0) # Hardware command sent
        self.current_task = 'drive_distance' # Thread begins monitoring

    def turn_in_place(self, angle_rad):
        current_h = self.world.get_imu()[0]
        self.task_params["target_angle"] = current_h + angle_rad
        
        # Positive angle = turn left, Negative angle = turn right
        turn_speed = 0.5 
        self.robot.set_velocity(0.3, turn_speed) # Hardware command sent
        self.current_task = 'turn' # Thread begins monitoring

    def drive_circle(self, linear_speed, angular_speed, duration):
        self.task_params["duration"] = duration
        self.task_params["start_time"] = time.time()
        
        self.robot.set_velocity(linear_speed, angular_speed) # Hardware command sent
        self.current_task = "drive_circle" # Thread begins monitoring
    
    def is_busy(self):
        """Returns True if the robot is currently executing a motion command."""
        return self.current_task is not None

    def stop(self):
        """Safely stops the motion controller thread and the robot."""
        self.running = False
        self.robot.stop()