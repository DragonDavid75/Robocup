from core.world_model import WorldModel
from core.robot_interface import RobotInterface
from mqtt_python.sir import ir
from mqtt_python.uservice import service
import time

print("Starting IR test...")

world = WorldModel()
robot = RobotInterface()

try:
    # Give system a moment to start
    time.sleep(1.0)

    while not service.stop:
        print(f"IR Left: {ir.ir[0]:.3f} m | IR Right: {ir.ir[1]:.3f} m")
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopping...")

robot.terminate()
