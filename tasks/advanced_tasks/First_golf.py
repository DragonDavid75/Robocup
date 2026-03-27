# tasks/ball_follow_and_drive.py

from tasks.base_task import BaseTask, TaskStatus
import time

class ForwardWithBallTask(BaseTask):

    def __init__(self, world, motion_controller, servo_controller):
        super().__init__(world, motion_controller, servo_controller)
        self.state = 0

    def start(self):
        super().start()
        print("[TASK] BallFollowAndDriveTask started")
        self.state = 0

    def update(self):
        # STATE 0: Follow the line for 7.5 meters
        if self.state == 0:
            print("[TASK] Following line for 7.5m")
            self.motion_controller.follow_for_distance(3, 0.6)
            self.state = 1

        # STATE 1: Wait for line follow to finish, then drive straight 2.7 meters
        elif self.state == 1:
            if not self.motion_controller.is_busy():
                print("[TASK] Driving straight for 2.7m")
                self.motion_controller.drive_distance(0.2, 0.4)
                self.state = 2
        
        elif self.state == 2:
            if not self.motion_controller.is_busy():
                self.motion_controller.turn_in_place(3.14)
                self.state = 3

        # STATE 2: Wait for drive to finish, then activate servo
        elif self.state == 3:
            if not self.motion_controller.is_busy():
                print("[TASK] Activating servo")
                self.servo_controller.servo_control(1, 200, 300)
                time.sleep(0.5)  # Small delay to ensure servo action completes
                self.servo_controller.servo_control(2, 0, 300)
                time.sleep(0.5)  # Small delay to ensure servo action completes
                self.servo_controller.servo_control(1, -800, 300)
                self.state = 4

        

        # STATE 3: Task Completion
        elif self.state == 4:
            print("[TASK] ForwardWithBallTask completed")
            return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        print("[TASK] BallFollowAndDriveTask stopped")