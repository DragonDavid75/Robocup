import threading
import time
import math
from mqtt_python.uservice import service


class DriveToPoint(threading.Thread):
    def __init__(self, world, target_x=1.0, target_y=1.0):
        super().__init__()
        self.world = world
        self.running = True

        self.target = (target_x, target_y)
        self.origin_pose = (0.0, 0.0, 0.0)

        self.MQTT_TOPIC_DRIVE = "robobot/cmd/ti"

    # ---------------------------------------------------
    def get_relative_pose(self):
        gx, gy, gh = self.world.get_pose()
        ox, oy, oh = self.origin_pose

        dx, dy = gx - ox, gy - oy

        cos_h, sin_h = math.cos(-oh), math.sin(-oh)

        rel_x = dx * cos_h - dy * sin_h
        rel_y = dx * sin_h + dy * cos_h
        rel_h = (gh - oh + math.pi) % (2 * math.pi) - math.pi

        return rel_x, rel_y, rel_h

    # ---------------------------------------------------
    def stop_robot(self, reason="Done"):
        self.running = False
        service.send(self.MQTT_TOPIC_DRIVE, "rc 0 0")
        print(f"\n[STOP] {reason}")

    # ---------------------------------------------------
    def run(self):
        self.origin_pose = self.world.get_pose()
        tx, ty = self.target

        try:
            while self.running:

                # ===============================
                # SENSOR INPUT
                # ===============================
                rx, ry, rh = self.get_relative_pose()

                dx = tx - rx
                dy = ty - ry
                dist = math.sqrt(dx * dx + dy * dy)

                desired_heading = math.atan2(dy, dx)

                heading_error = desired_heading - rh

                # wrap to [-pi, pi]
                if heading_error > math.pi:
                    heading_error -= 2 * math.pi
                elif heading_error < -math.pi:
                    heading_error += 2 * math.pi

                # ===============================
                # DEBUG OUTPUT
                # ===============================
                print(
                    f"[DriveToPoint] "
                    f"x={rx:.3f}, y={ry:.3f}, h={math.degrees(rh):.3f} | "
                    f"dist={dist:.3f} | "
                    f"des_h={math.degrees(desired_heading):.3f} | "
                    f"err={math.degrees(heading_error):.3f}"
                )

                # ===============================
                # SAFE OUTPUT ONLY (NO CONTROL)
                # ===============================
                service.send(self.MQTT_TOPIC_DRIVE, "rc 0 0")

                time.sleep(0.1)

        except Exception as e:
            print(f"[ERROR] {e}")

        finally:
            self.stop_robot("Loop exited")
