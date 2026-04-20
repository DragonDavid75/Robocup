# core/mission_manager.py

import time
from tasks.vivek.ball import BallTask
from tasks.base_task import TaskStatus
from tasks.base_tasks.drive_dist_line import DriveDistLineTask
from tasks.base_tasks.small_turn import SmallTurnTask
from tasks.advanced_tasks.bonus_time import BonusTimeTask
from tasks.advanced_tasks.timer_first import TimerFirstTask
from tasks.advanced_tasks.start_roundabout import StartRoundaboutTask
from tasks.advanced_tasks.first_ball import FirstBallTask
from tasks.advanced_tasks.test_task import TestTask
from tasks.advanced_tasks.first_golf import Firstball
from tasks.drive_roundabout import DriveRoundaboutTask
from tasks.exit_roundabout import ExitRoundaboutTask
from tasks.nil.drive_to_hole_task import DriveToHolePointTask

class MissionManager:

    def __init__(self, world, motion_controller, servo_controller):
        self.world = world
        self.motion_controller = motion_controller
        self.servo_controller = servo_controller
        self.current_task = None
        self.task_queue = []
        self.running = True

        # Define mission sequence here
        self.build_mission()

    """
    Builds the mission by adding tasks to the task queue in the desired order.
    You can customize this method to create different mission sequences by adding or removing tasks.
    Each task should be an instance of a class that implements the Task interface (with start, update, and stop methods).
    """
    def build_mission(self):
        #self.task_queue.append(BallTask(self.world, self.motion_controller, self.servo_controller))
        #self.task_queue.append(Firstball(self.world, self.motion_controller, self.servo_controller, distance=1.0, velocity=0.2))
        # self.task_queue.append(DriveToPointTask(self.world, self.motion_controller, self.servo_controller, target_x=1.0, target_y=1.0))  # Example: drive 1m forward
        self.task_queue.append(DriveToHolePointTask(self.world, self.motion_controller, self.servo_controller))

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
