from core.world_model import WorldModel
from core.robot_interface import RobotInterface
from core.motion_controller import MotionController
from core.servo_controller import ServoController
from core.vision_system import VisionSystem
from core.mission_manager import MissionManager
from core.localizer import Localizer
from core.line_following import LineFollower
import time
from mqtt_python.uservice import service
from core.drive_to_ball import DriveToBallTask

world = WorldModel()
robot = RobotInterface()
line_follower = LineFollower()
drive_to_ball = DriveToBallTask(world)

motion = MotionController(world, robot, line_follower, drive_to_ball)
servo = ServoController(robot)
vision = VisionSystem(world)
mission = MissionManager(world, motion, servo)
localizer = Localizer(world)

motion.start()
servo.start()
vision.start()
localizer.start()
line_follower.start()

try:
    while not service.stop and True:
        mission.update()
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Shutting down...")

print("Stopping threads...")
servo.stop()
motion.stop()
vision.stop()
localizer.stop()
line_follower.stop()
robot.terminate()