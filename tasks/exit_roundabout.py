# tasks/exit_roundabout.py

from tasks.base_task import BaseTask, TaskStatus
from mqtt_python.spose import pose


class ExitRoundaboutTask(BaseTask):

    def __init__(self, world, motion_controller):
        super().__init__(world, motion_controller)
        self.state = 0   # State machine variable

    def start(self):
        super().start()
        print("[TASK] ExitRoundabout started")
        self.state = 0   # Reset state when task starts

    def update(self):
        """
        State machine:
        0 -> Turn towards exit
        1 -> Wait for turn to finish
        2 -> Start line following
        3 -> Wait for line following to finish
        4 -> Ensure robot fully stopped
        """

        if self.state == 0:
            # Step 1: Turn towards the expected exit direction
            # -1.57 rad ≈ 90 degrees to the right
            # Change this depending on which exit you want
            self.motion_controller.turn_in_place(-1.57)
            self.state = 1   # Move to next state

        elif self.state == 1:
            # Step 2: Wait until turning is finished
            # motion_controller.is_busy() == True → still turning
            if not self.motion_controller.is_busy():
                self.state = 2

        elif self.state == 2:
            # Step 3: Start following the line after exiting roundabout
            # left_side=True means robot follows the left edge of the line
            self.motion_controller.follow_until_intersection_or_end_line(
                0.25,          # Speed (m/s)
                left_side=True,
                ref_pos=0.0
            )
            self.state = 3

        elif self.state == 3:
            # Step 4: Wait until line following is done
            # This typically ends at intersection or end of line
            if not self.motion_controller.is_busy():
                self.state = 4

        elif self.state == 4:
            # Step 5: Ensure robot has fully stopped before finishing task
            if abs(pose.velocity()) < 0.001:
                print("[TASK] ExitRoundabout completed")
                return TaskStatus.DONE

        # Task still running
        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        print("[TASK] ExitRoundabout stopped")
