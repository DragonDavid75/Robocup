import math
import time

from tasks.base_task import BaseTask, TaskStatus


class DropGolfTask(BaseTask):
    def __init__(self, world, motion_controller, servo_controller):
        super().__init__(world, motion_controller, servo_controller)

        self.servo_controller = servo_controller
        self.state = 0

    def start(self):
        super().start()
        print("[TASK] Simple DropGolfTask started")
        self.state = 0

    def update(self):
        # Step 0: drive to the first intersection
        if self.state == 0:
            print("[TASK] Driving to the first intersection")
            self.servo_controller.servo_control(1, -800, 300)
            self.motion_controller.follow_until_intersection_or_end_line(0.4)
            self.state = 1
        elif self.state == 1:
            if not self.motion_controller.is_busy():
                self.state = 2
        # Step 1: Reach the first intersection and pass it
        elif self.state == 2:
            print("[TASK] Driving forward 0.1 m")
            self.motion_controller.follow_for_distance(0.57, 0.4)
            self.state = 3
        elif self.state == 3:
            if not self.motion_controller.is_busy():
                print("[TASK] Going")
                self.state = 4
        # Step 2: Turn right at intersection
        elif self.state == 4:
            self.motion_controller.turn_in_place(math.radians(-90))
            self.state = 5
        elif self.state == 5:
            if not self.motion_controller.is_busy():
                print("[TASK] Turn finished")
                self.state = 6
        #Step 3: Drive up the ramp
        elif self.state == 6:
            print("[TASK] Driving forward 0.1 m")
            self.motion_controller.follow_for_distance(3.8, 0.4)
            self.state = 7
        elif self.state == 7:
            if not self.motion_controller.is_busy():
                print("[TASK] At the ramp")
                self.state = 8
        #Step 4: Locate the hole
        elif self.state == 8:
            print("[TASK] Driving forward 0.1 m")
            self.motion_controller.drive_distance(0.2, 0.1)
            self.state = 9
        elif self.state == 9:
            if not self.motion_controller.is_busy():
                print("[TASK] Next to the hole")
                self.state = 10
        #Step 4: Lower the arm and drop the golf ball
        elif self.state == 10:
            if not self.motion_controller.is_busy():
                self.servo_controller.servo_control(1, 200, 500) #lower arm
                time.sleep(0.5)
                self.state = 11
        elif self.state == 11:
            if not self.motion_controller.is_busy():
                print("[TASK] Lowered arm")
                self.state = 12
        #Step 5: Rotate to left to drop the ball in the hole
        elif self.state == 12:
            self.motion_controller.turn_in_place(math.radians(105))
            self.state = 13
        elif self.state == 13:
            if not self.motion_controller.is_busy():
                print("[TASK] Rotates")
                self.state = 14
        elif self.state == 14:
            if not self.motion_controller.is_busy():
                self.servo_controller.servo_control(2, 0, 300) #open arm
                time.sleep(0.5)
                self.state = 15
        elif self.state == 15:
            if not self.motion_controller.is_busy():
                print("[TASK] Opened arm")
                self.state = 16
        elif self.state == 16:
            print("[TASK] DropGolfTask completed")
            return TaskStatus.DONE
               
        return TaskStatus.RUNNING

    def stop(self):
            super().stop()
            self.motion_controller.stop()
            print("[TASK] DropGolfTask stopped")