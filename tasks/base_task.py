# tasks/base_task.py

class TaskStatus:
    RUNNING = 0
    DONE = 1
    FAILED = 2


class BaseTask:

    def __init__(self, world, robot):
        self.world = world
        self.robot = robot
        self.started = False

    def start(self):
        """
        Called once when task begins.
        """
        self.started = True

    def update(self):
        """
        Called repeatedly by mission manager.
        Must return TaskStatus.
        """
        return TaskStatus.RUNNING

    def stop(self):
        """
        Called once when task finishes or is aborted.
        """
        self.world.set_motion(0, 0)
