import math
import time

from tasks.base_task import BaseTask, TaskStatus


class DriveToGolf1Task(BaseTask):
    def __init__(self, world, motion_controller, servo_controller):
        super().__init__(world, motion_controller, servo_controller)

        self.servo_controller = servo_controller
        self.state = 0

    def start(self):
        super().start()
        print("[TASK] Simple DriveToGolf1Task started")
        self.state = 0

    def update(self):
        # Step 0: drive forward 0.25 m at speed 0.1
        if self.state == 0:
            print("[TASK] Driving forward 0.25 m")
            self.motion_controller.follow_for_distance(0.3, 0.1)
            self.state = 1

        elif self.state == 1:
            if not self.motion_controller.is_busy():
                self.state = 4

        # # Step 1: lower gripper arm
        # elif self.state == 2:
        #     print("[TASK] Lowering gripper arm")
        #     self.servo_controller.servo_control(1, 210, 300)
        #     time.sleep(0.5)
        #     self.state = 3

        # # Step 2: open gripper
        # elif self.state == 3:
        #     print("[TASK] Opening gripper")
        #     self.servo_controller.servo_control(2, 0, 300)
        #     time.sleep(0.5)
        #     self.state = 4

        # Step 3: drive another 0.05 m at speed 0.1
        elif self.state == 4:
            print("[TASK] Driving forward 0.05 m")
            self.motion_controller.follow_for_distance(0.1, 0.05)
            self.state = 5

        elif self.state == 5:
            if not self.motion_controller.is_busy():
                self.state = 6

        # Step 4: close gripper
        elif self.state == 6:
            print("[TASK] Closing gripper")
            self.servo_controller.servo_control(2, -1000, 300)
            time.sleep(1.0)
            self.state = 7

        elif self.state == 7:
                self.motion_controller.follow_for_distance(0.47, 0.1)
                self.state = 8

        elif self.state == 8:
            if not self.motion_controller.is_busy():
                self.state = 9

        elif self.state == 9:
            # lift the servo arm 
            self.servo_controller.servo_control(1, -500, 300)
            self.state = 10

        elif self.state == 10:
                self.motion_controller.follow_for_distance(0.5, 0.1)
                self.state = 11
        
        elif self.state == 11:
            if not self.motion_controller.is_busy():
                self.state = 12

        # elif self.state == 12:
        #     print("[TASK] turn right on the intersection")
        #     self.motion_controller.drive_distance(1.0,0.4)
        #     self.state = 13

        # elif self.state == 5:
        #     if not self.motion_controller.is_busy():
        #         self.motion_controller.turn_in_place(-1.57)
        #         self.state = 6

        elif self.state == 12:
            print("[TASK] DriveToPoint completed")
            return TaskStatus.DONE

            # elif self.state == 12:
            #     return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
            super().stop()
            self.motion_controller.stop()
            print("[TASK] DriveToPoint stopped")