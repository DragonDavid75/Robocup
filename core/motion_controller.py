# core/motion_controller.py
import threading
import time

class MotionController(threading.Thread):

    def __init__(self, world, robot):
        super().__init__()
        self.world = world
        self.robot = robot
        self.running = True

    def run(self):
        while self.running:
            v, w = self.world.get_motion()
            self.robot.send_velocity(v, w)
            time.sleep(0.05)  # 20 Hz

    def stop(self):
        self.running = False
        self.robot.stop()
