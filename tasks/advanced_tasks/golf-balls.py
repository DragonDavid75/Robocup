#Third of the tasks
#It starts before the junction after the ramp
#Goes to the junction and turns left and goes to the ball
#Finds the first orange golf ball, picks it up
#next steps TBD

from tasks.base_task import BaseTask, TaskStatus
from tasks.base_tasks.drive_dist import DriveDistTask
from tasks.base_tasks.drive_dist_line import DriveDistLineTask
from mqtt_python.spose import pose
import time


class GolfBallsTask(BaseTask):

    def __init__(self, world, motion_controller, servo_controller):
        super().__init__(world, motion_controller, servo_controller)
        self.state = 0

    def start(self):
        super().start()
        print("[TASK] GolfBallsTask started")
        self.state = 0

    def update(self):
        #STATE 0: Follow line until junction, then turn left
        if self.state == 0:
            self.servo_controller.servo_control(1, -800, 300)
            self.motion_controller.follow_for_distance(0.3, 0.6, action="LEFT")
            self.state = 1
        #STATE 1: Drive to ball
        elif self.state == 1:
            if not self.motion_controller.is_busy():
                self.motion_controller.follow_for_distance(1, 0.2)
                self.state = 2
        #STATE 2: Pick up ball
        #Drive forward with the arm lowered slowly
        #Raise arm up after aroound 0.7 m
        #Continue straingt until white line is found
        #Turn 90 degrees to the right and follow the line unntil the next intersection
        #Continue straight until off the ramp
        #Go to the hoe
        #Drop off the first golf ball
        #Maybe all things with the second golf ball should be in a seperate task?

        elif self.state == 2:
            if not self.motion_controller.is_busy():
                print("[TASK] GolfBallsTask completed")
                return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        print("[TASK] GolfBallsTask stopped")
