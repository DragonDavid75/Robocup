# tasks/robot_primitives.py
# Reusable robot primitives: drive, rotate, wait_until_stopped, record_pose

import math
import time


def record_pose(world):
    """Record current pose as reference."""
    return world.get_pose()


def drive(world, robot, distance, speed=0.2, start_x=None, start_y=None):
    """
    Drive forward a specified distance in meters.
    Returns True when target reached.
    
    Args:
        world: WorldModel instance
        robot: Robot interface
        distance: distance in meters
        speed: forward speed m/s
        start_x, start_y: reference position (if None, uses current position)
    """
    x, y, _ = world.get_pose()
    
    if start_x is None:
        start_x, start_y, _ = world.get_pose()
    
    dist = ((x - start_x)**2 + (y - start_y)**2)**0.5
    
    if dist < distance:
        world.set_motion(speed, 0.0)
        robot.set_servo(1, -800, 300)
        return False, start_x, start_y
    else:
        world.set_motion(0.0, 0.0)
        robot.set_servo(1, 0, 0)
        return True, start_x, start_y


def rotate(world, angle_deg, turn_rate=1.5, start_h=None):
    """
    Rotate a specified angle in degrees.
    Returns True when target reached.
    
    Args:
        world: WorldModel instance
        angle_deg: angle in degrees (positive = left, negative = right)
        turn_rate: angular velocity rad/s
        start_h: reference heading (if None, uses current heading)
    """
    _, _, h = world.get_pose()
    
    if start_h is None:
        _, _, start_h = world.get_pose()
    
    delta_h = abs(h - start_h)
    # Normalize to [0, 2pi]
    delta_h = delta_h % (2 * math.pi)
    if delta_h > math.pi:
        delta_h = 2 * math.pi - delta_h
    
    angle_rad = math.radians(abs(angle_deg))
    
    if delta_h < angle_rad:
        direction = 1 if angle_deg > 0 else -1
        world.set_motion(0.0, turn_rate * direction)
        return False, start_h
    else:
        world.set_motion(0.0, 0.0)
        return True, start_h


def wait_until_stopped():
    """
    Wait for robot to come to a complete stop.
    Returns True when stopped.
    """
    from mqtt_python.spose import pose
    return pose.velocity() < 0.001
