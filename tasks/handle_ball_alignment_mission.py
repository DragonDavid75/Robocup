#tasks/handle_ball_alignment_mission.py

import time
from tasks.base_task import BaseTask, TaskStatus

class AlignToBallTask(BaseTask):
    def __init__(self, world, motion_controller, color="blue", timeout=15.0):
        """
        :param color: The ball color to target ('blue', 'red', 'green', 'white')
        :param timeout: Max time (seconds) to search before failing
        """
        super().__init__(world, motion_controller)
        self.color = color
        self.timeout = timeout
        self.start_time = 0

    def start(self):
        """Initializes the alignment in the MotionController."""
        print(f"[TASK] Starting alignment mission for {self.color} ball...")
        self.start_time = time.time()
        # Trigger the specialized handler we added to MotionController
        self.motion_controller.align_to_ball(self.color)

    def update(self):
        """Checked 20 times per second by the MissionManager."""
        
        # 1. Check if MotionController finished the job (is_busy becomes False)
        if not self.motion_controller.is_busy():
            print(f"[TASK] Alignment Successful: {self.color} ball centered.")
            return TaskStatus.DONE

        # 2. Check for Timeout (Safety measure)
        if (time.time() - self.start_time) > self.timeout:
            print(f"[TASK] FAILED: Could not find/align to {self.color} ball in {self.timeout}s")
            self.motion_controller.stop()
            return TaskStatus.FAILED

        # 3. Otherwise, we are still spinning/searching
        return TaskStatus.RUNNING

    def stop(self):
        """Emergency or normal stop."""
        self.motion_controller.stop()