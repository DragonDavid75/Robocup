from core.world_model import WorldModel
from core.robot_interface import RobotInterface
from core.motion_controller import MotionController
from core.vision_system import VisionSystem
from core.mission_manager import MissionManager
from core.localizer import Localizer
from core.visualizer import TrackVisualizer
import time

world = WorldModel()
robot = RobotInterface()

motion = MotionController(world, robot)
vision = VisionSystem(world)
localizer = Localizer(world)
visualizer = TrackVisualizer(world)
mission = MissionManager(world, robot)

motion.start()
vision.start()
localizer.start()
visualizer.start()

try:
    while True:
        mission.update()
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Shutting down...")

motion.stop()
vision.stop()
localizer.stop()
visualizer.stop()
robot.terminate()