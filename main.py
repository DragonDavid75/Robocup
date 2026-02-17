from core.world_model import WorldModel
from core.robot_interface import RobotInterface
from core.motion_controller import MotionController
from core.vision_system import VisionSystem
from core.mission_manager import MissionManager
# import required code from core
# each python code in code does its respective tasks
# each task run forever in a while true loop on a seperate thread
# all the threads are run in paralle, use the existing mqtt server to communicate between the threads
# follow the naming convention for easy communication. 
# topic "internal/thread<threadnumber>/<task>/<variable as required>"
# ex: "internal/thread2/computervision/getGolfBall"
# ex: "internal/thread2/computervision/getArucoCode"
# ex: "internal/thread2/computervision/getArucoPosition"

import threading
import time

world = WorldModel()
robot = RobotInterface()

motion = MotionController(world, robot)
vision = VisionSystem(world)
mission = MissionManager(world, robot)

def thread1_func():
    while True:
        print("Thread 1 running...")
        #assign thread 1 to anything u need to
        time.sleep(2)

def thread2_func():
    while True:
        print("Thread 2 running...")
        #assign thread 2 to anything u need to
        time.sleep(3)

def main_loop():
    while True:
        print("Main thread running...")
        #this is the main thread,
        #the decision making and keeping track of the robot happens in this thread
        #make sure to not spawn too many new threads, limit the maximum number of threads to 4, including the main program.
        time.sleep(4)

# Create threads
t1 = threading.Thread(target=thread1_func, daemon=True)
t2 = threading.Thread(target=thread2_func, daemon=True)

# Start threads
t1.start()
t2.start()

# Run main thread loop (third parallel loop)
main_loop()
