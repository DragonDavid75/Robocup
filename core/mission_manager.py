# core/mission_manager.py

import time
from tasks.drive_one_meter import DriveOneMeterTask
from tasks.base_task import TaskStatus


class MissionManager:

    def __init__(self, world, robot):
        self.world = world
        self.robot = robot
        self.current_task = None
        self.task_queue = []
        self.running = True

        # Define mission sequence here
        self.build_mission()

    def build_mission(self):
        """
        Define the mission sequence.
        This is where your group defines order of tasks.
        """
        self.task_queue.append(DriveOneMeterTask(self.world, self.robot))

    def start_next_task(self):
        if len(self.task_queue) == 0:
            print("[MISSION] All tasks completed")
            self.running = False
            return

        self.current_task = self.task_queue.pop(0)
        self.current_task.start()

    def update(self):

        if not self.running:
            return

        # Start first task if none running
        if self.current_task is None:
            self.start_next_task()
            return

        # Update current task
        status = self.current_task.update()

        if status == TaskStatus.DONE:
            self.current_task.stop()
            self.current_task = None

        elif status == TaskStatus.FAILED:
            print("[MISSION] Task failed")
            self.current_task.stop()
            self.running = False
