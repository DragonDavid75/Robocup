import math
from tasks.base_task import BaseTask, TaskStatus

class ObstacleAvoidanceTask(BaseTask):
    """
    Task to drive forward, perform an arc to avoid an obstacle, 
    and then turn in place.
    """

    def __init__(self, world, motion_controller, servo_controller):
        super().__init__(world, motion_controller, servo_controller)
        self.state = 0

    def start(self):
        super().start()
        print("[TASK] ObstacleAvoidanceTask started")
        self.state = 0

    def update(self):
        # State 0: Drive forward 0.5m
        if self.state == 0:
            print("[TASK] Driving forward...")
            self.motion_controller.drive_distance(0.5, 0.6)
            self.state = 1

        # State 1: Wait for forward drive to finish
        elif self.state == 1:
            if not self.motion_controller.is_busy():
                print("[TASK] Performing arc maneuver...")
                self.motion_controller.drive_arc(
                    angle_rad=math.radians(45),
                    radius=0.20,
                    linear_speed=0.09
                )
                self.state = 2

        # State 2: Wait for arc to finish
        elif self.state == 2:
            if not self.motion_controller.is_busy():
                print("[TASK] Turning in place...")
                self.motion_controller.turn_in_place(-1.9)
                self.state = 3

        # State 3: Wait for turn in place to finish
        elif self.state == 3:
            if not self.motion_controller.is_busy():
                print("[TASK] ObstacleAvoidanceTask completed")
                return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        print("[TASK] ObstacleAvoidanceTask stopped")