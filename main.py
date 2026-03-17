from core.world_model import WorldModel
from core.robot_interface import RobotInterface
from core.motion_controller import MotionController
from core.vision_system import VisionSystem
from core.mission_manager import MissionManager
import time
from mqtt_python.uservice import service

world = WorldModel()
robot = RobotInterface()

motion = MotionController(world, robot)
vision = VisionSystem(world)
mission = MissionManager(world, motion)

motion.start()
vision.start()

try:
    while not service.stop and mission.running:
        mission.update()
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Shutting down...")

print("Stopping motion and vision threads...")
motion.stop()
vision.stop()
robot.terminate()