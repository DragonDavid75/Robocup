# core/mission_manager.py

import time
from tasks.base_task import TaskStatus
from tasks.base_tasks.drive_dist_line import DriveDistLineTask
from tasks.base_tasks.small_turn import SmallTurnTask
from tasks.advanced_tasks.bonus_time import BonusTimeTask
from tasks.advanced_tasks.timer_first import TimerFirstTask
from tasks.advanced_tasks.first_ball import FirstBallTask
from tasks.advanced_tasks.test_task import TestTask
from tasks.advanced_tasks.first_golf import ForwardWithBallTask
from tasks.drive_roundabout import DriveRoundaboutTask
from tasks.exit_roundabout import ExitRoundaboutTask

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

    def build_mission(self):
        """
        Define the mission sequence.
        This is where your group defines order of tasks.
        """
        # self.task_queue.append(ForwardWithBallTask(self.world, self.motion_controller, self.servo_controller))
        # self.task_queue.append(DriveRoundaboutTask(self.world, self.motion_controller, self.servo_controller))
        # Exit roundabout: turn right 90° and follow the line
        # self.task_queue.append(ExitRoundaboutTask(self.world,self.motion_controller,self.servo_controller))
        # self.task_queue.append(DriveDistLineTask(self.world, self.motion_controller, self.servo_controller, distance=50.0, velocity=0.4))
        # self.task_queue.append(SmallTurnTask(self.world, self.motion_controller, self.servo_controller, velocity=0.4, action="RIGHT"))
        #self.task_queue.append(TimerFirstTask(self.world, self.motion_controller, self.servo_controller))
        #self.task_queue.append(FirstBallTask(self.world, self.motion_controller, self.servo_controller))

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
