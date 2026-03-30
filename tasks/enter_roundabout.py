
# tasks/enter_roundabout_task.py

from tasks.base_task import BaseTask, TaskStatus
import time


class EnterRoundaboutTask(BaseTask):
    def __init__(self, world, motion_controller):
        super().__init__(world, motion_controller)

        # States:
        # 0 = first forward move
        # 1 = wait
        # 2 = turn toward roundabout
        # 3 = wait
        # 4 = second forward move into roundabout
        # 5 = wait
        # 6 = done
        self.state = 0

        # First forward distance before turning
        self.approach_distance = 0.20
        self.approach_speed = 0.15

        # Turn angle toward the roundabout
        # Positive or negative depends on your controller convention
        self.turn_angle_deg = 35
        self.turn_speed = 0.3

        # Final forward move into the circle
        self.enter_distance = 0.20
        self.enter_speed = 0.12

    def start(self):
        super().start()
        print("[TASK] EnterRoundabout started")
        self.state = 0

    def update(self):
        if self.state == 0:
            print("[TASK] Approaching roundabout")
            self.motion_controller.drive_distance(
                self.approach_distance,
                self.approach_speed
            )
            self.state = 1

        elif self.state == 1:
            if not self.motion_controller.is_busy():
                print("[TASK] Turning toward roundabout")
                self.motion_controller.turn_angle(
                    self.turn_angle_deg,
                    self.turn_speed
                )
                self.state = 2

        elif self.state == 2:
            if not self.motion_controller.is_busy():
                print("[TASK] Entering roundabout")
                self.motion_controller.drive_distance(
                    self.enter_distance,
                    self.enter_speed
                )
                self.state = 3

        elif self.state == 3:
            if not self.motion_controller.is_busy():
                print("[TASK] EnterRoundabout completed")
                return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        self.motion_controller.stop()
        print("[TASK] EnterRoundabout stopped")
         

