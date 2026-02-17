from core.world_model import WorldModel
from core.robot_interface import RobotInterface
from core.motion_controller import MotionController
from core.vision_system import VisionSystem
from core.mission_manager import MissionManager
import time

world = WorldModel()
robot = RobotInterface()

motion = MotionController(world, robot)
vision = VisionSystem(world)
mission = MissionManager(world, robot)

motion.start()
vision.start()

try:
    while True:
        mission.update()
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Shutting down...")

motion.stop()
vision.stop()
robot.terminate()