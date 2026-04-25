# core/mission_manager.py

import time
from tasks.base_task import TaskStatus
from tasks.advanced_tasks.a_start_roundabout import StartRoundaboutTask
from tasks.advanced_tasks.b_timer_first import TimerFirstTask
from tasks.advanced_tasks.c_get_golf_1 import DriveToGolf1Task
from tasks.advanced_tasks.d_drop_golf_1 import DriveToHole1Task
from tasks.advanced_tasks.e_get_golf_2 import DriveToGolf2Task
from tasks.advanced_tasks.f_drop_golf_2 import DriveToHole2Task
from tasks.advanced_tasks.g_down_stairs import DriveDownStairs



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
        self.task_queue.append(StartRoundaboutTask(self.world, self.motion_controller, self.servo_controller))
        self.task_queue.append(TimerFirstTask(self.world, self.motion_controller, self.servo_controller))
        self.task_queue.append(DriveToGolf1Task(self.world, self.motion_controller, self.servo_controller, 'golf'))
        self.task_queue.append(DriveToHole1Task(self.world, self.motion_controller, self.servo_controller))
        self.task_queue.append(DriveToGolf2Task(self.world, self.motion_controller, self.servo_controller, 'golf'))
        self.task_queue.append(DriveToHole2Task(self.world, self.motion_controller, self.servo_controller))
        self.task_queue.append(DriveDownStairs(self.world, self.motion_controller, self.servo_controller))

    
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
