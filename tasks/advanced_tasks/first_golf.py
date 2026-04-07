# tasks/first_golf.py

from tasks.base_task import BaseTask, TaskStatus
import time

class Firstball(BaseTask):

    def __init__(self, world, motion_controller, servo_controller, distance=1.0, velocity=0.2):
        super().__init__(world, motion_controller, servo_controller)
        self.state = 0
        self.distance = distance
        self.velocity = velocity

    def start(self):
        super().start()
        print("[TASK] BallFollowAndDriveTask started")
        self.state = 1

    def update(self):
        # STATE 0: Drive to the ball using the vision-based system
        if self.state == 1:
            if not self.motion_controller.is_busy():
                self.servo_controller.servo_control(1, -800, 300)
                print("[TASK] Ball reached. Now following line for 2.5m")
                self.motion_controller.follow_for_distance(2.5, 0.6)
                self.state = 2

        # STATE 2: Wait for line follow (2.5m) to finish
        elif self.state == 2:
            if not self.motion_controller.is_busy():
                print("[TASK] Line follow complete. Driving straight for 0.5m")
                self.motion_controller.drive_distance(0.5, 0.6)
                self.state = 3  

        # STATE 3: Wait for drive distance (0.5m) to finish
        elif self.state == 3:
            if not self.motion_controller.is_busy():
                print("[TASK] Driving complete. Activating servos...")
                # Move servo 1 to 200
                self.servo_controller.servo_control(1, 200, 300)
                self.state = 4

        # STATE 4: Wait for first servo movement, then move servo 2
        elif self.state == 4:
            if not self.servo_controller.is_busy():
                self.servo_controller.servo_control(2, 0, 300)
                self.state = 5

        # STATE 5: Wait for second servo movement, then reset servo 1
        elif self.state == 5:
            if not self.servo_controller.is_busy():
                self.servo_controller.servo_control(1, -800, 300)
                self.state = 6.5
        #state 6.5: turn around on your position 45 degrees
        elif self.state == 6.5:
            if not self.servo_controller.is_busy():
                print("[TASK] Servos activated. Turning around...")
                self.motion_controller.turn_in_place(-1.9)  # Turn 45 degrees
                self.state = 7.5

        elif self.state == 7.5:
            if not self.motion_controller.is_busy():
                print("[TASK] Turn complete. Driving to ball again...")
                self.state = 8

        # STATE 7: go to ball
        elif self.state == 8:
            if not self.servo_controller.is_busy():
                print("[TASK] Servos activated. Driving to ball again...")
                self.motion_controller.drive_to_ball()
                self.state = 9
        
        elif self.state == 9:
            if not self.servo_controller.is_busy():
                print("[TASK] BallFollowAndDriveTask completed successfully")
                return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        print("[TASK] BallFollowAndDriveTask stopped")