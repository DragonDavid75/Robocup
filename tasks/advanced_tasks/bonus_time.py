# tasks/bonus_time.py

from tasks.base_task import BaseTask, TaskStatus
from tasks.base_tasks.drive_dist import DriveDistTask
from tasks.base_tasks.drive_dist_line import DriveDistLineTask
from mqtt_python.spose import pose
import time


class BonusTimeTask(BaseTask):

    def __init__(self, world, motion_controller, servo_controller):
        super().__init__(world, motion_controller, servo_controller)
        self.state = 0

    def start(self):
        super().start()
        print("[TASK] BonusTimeTask started")
        self.state = 0

    def update(self):
        if self.state == 0:
            print("[TASK] BonusTimeTask: State 0")
            # Start following the line
            self.servo_controller.servo_control(1, -800, 300)
            self.sub_task = DriveDistLineTask(self.world, self.motion_controller, self.servo_controller, distance=4.5, velocity=0.2)
            self.sub_task.start()
            self.state = 1

        elif self.state == 1:
            print("[TASK] BonusTimeTask: State 1")
            status = self.sub_task.update()
            if status == TaskStatus.DONE:
                self.sub_task.stop()
                self.sub_task = DriveDistTask(self.world, self.motion_controller, self.servo_controller, distance=2.2, velocity=-0.2)
                self.state = 2

        elif self.state == 2:
            print("[TASK] BonusTimeTask: State 2")
            status = self.sub_task.update()
            if status == TaskStatus.DONE:
                self.sub_task.stop()
                self.sub_task = DriveDistTask(self.world, self.motion_controller, self.servo_controller, distance=2.2, velocity=0.2)
                self.state = 3

        elif self.state == 3:
            print("[TASK] BonusTimeTask: State 3")
            status = self.sub_task.update()
            if status == TaskStatus.DONE:
                self.sub_task.stop()
                self.sub_task = DriveDistLineTask(self.world, self.motion_controller, self.servo_controller, distance=5.0, velocity=0.2)
                self.state = 4

        elif self.state == 4:
            print("[TASK] BonusTimeTask: State 4")
            status = self.sub_task.update()
            if status == TaskStatus.DONE:
                self.sub_task.stop()
                print("[TASK] BonusTimeTask completed")
                return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        print("[TASK] BonusTimeTask stopped")
