# core/localizer.py
import threading
import time
import numpy as np

# Set USE_LOCALIZER = True to enable full localization with IMU correction
# Set to False for simple pose bridge (just copy odometry to WorldModel)
USE_LOCALIZER = False

# Map reference point (where robot starts in world coordinates)
# Adjust these to match your reference map
MAP_OFFSET_X = 0.0
MAP_OFFSET_Y = 0.0
MAP_OFFSET_H = 0.0  # heading offset (radians)


class Localizer(threading.Thread):
    def __init__(self, world):
        super().__init__()
        self.world = world
        self.running = True
        
        # Odometry tracking for dead reckoning
        self.last_pose_time = None
        self.last_odom_x = 0.0
        self.last_odom_y = 0.0
        self.last_odom_h = 0.0
        
        # IMU-based heading correction
        self.imu_heading_offset = 0.0  # set on first IMU reading
        self.integrated_heading = 0.0  # gyro-integrated heading
        self.last_gyro_time = None
        
    def run(self):
        while self.running:
            if USE_LOCALIZER:
                self._update_localized_pose()
            else:
                self._update_simple_pose()
            time.sleep(0.01)  # ~100 Hz update rate
    
    def _update_simple_pose(self):
        """Simply copy odometry pose to WorldModel"""
        from mqtt_python.spose import pose
        from mqtt_python.simu import imu
        
        print(f"[Localizer] poseCnt: {pose.poseCnt}, gyroUpdCnt: {imu.gyroUpdCnt}")
        
        if pose.poseCnt > 0:
            print(f"[Localizer] Pose: x={pose.pose[0]:.3f}, y={pose.pose[1]:.3f}, h={pose.pose[2]:.3f}")
            self.world.set_pose(pose.pose[0], pose.pose[1], pose.pose[2])
        
        if imu.gyroUpdCnt > 0:
            print(f"[Localizer] Gyro z: {imu.gyro[2]:.3f}")
            self.world.set_imu(self.integrated_heading, imu.gyro[2])
    
    def _update_localized_pose(self):
        """Dead reckoning with IMU heading correction"""
        from mqtt_python.spose import pose
        from mqtt_python.simu import imu
        
        print(f"[Localizer] poseCnt: {pose.poseCnt}, gyroUpdCnt: {imu.gyroUpdCnt}")
        
        # Get odometry from Teensy
        if pose.poseCnt == 0:
            return
        
        odom_x = pose.pose[0]
        odom_y = pose.pose[1]
        odom_h = pose.pose[2]
        
        print(f"[Localizer] Odometry: x={odom_x:.3f}, y={odom_y:.3f}, h={odom_h:.3f}")
        
        # Integrate gyro for heading (if IMU available)
        if imu.gyroUpdCnt > 0:
            gyro_z = imu.gyro[2]
            now = time.time()
            
            if self.last_gyro_time is not None:
                dt = now - self.last_gyro_time
                self.integrated_heading += gyro_z * dt
            
            self.last_gyro_time = now
            
            # Try to get euler from IMU if available, otherwise use integrated
            if hasattr(imu, 'euler') and imu.accUpdCnt > 0:
                imu_heading = imu.euler[2]  # yaw from IMU
            else:
                imu_heading = self.integrated_heading
            
            # Initialize IMU offset on first reading
            if self.imu_heading_offset == 0.0 and pose.poseCnt > 5:
                self.imu_heading_offset = imu_heading - odom_h
            
            # Apply IMU correction to heading
            corrected_h = imu_heading - self.imu_heading_offset
            
            # Normalize heading to [-pi, pi]
            corrected_h = np.arctan2(np.sin(corrected_h), np.cos(corrected_h))
            
            self.world.set_imu(imu_heading, gyro_z)
        else:
            corrected_h = odom_h
        
        # Apply map offset (transform from odometry to map coordinates)
        map_x = odom_x + MAP_OFFSET_X
        map_y = odom_y + MAP_OFFSET_Y
        
        print(f"[Localizer] Map coords: x={map_x:.3f}, y={map_y:.3f}, h={corrected_h:.3f}")
        
        self.world.set_pose(map_x, map_y, corrected_h)
    
    def reset_pose(self, x=0.0, y=0.0, h=0.0):
        """Reset localization to a known position"""
        self.last_odom_x = x
        self.last_odom_y = y
        self.last_odom_h = h
        self.imu_heading_offset = 0.0
        self.integrated_heading = 0.0
    
    def calibrate_imu_offset(self):
        """Recalibrate IMU heading offset (call when robot is stationary and facing known direction)"""
        from mqtt_python.spose import pose
        from mqtt_python.simu import imu
        
        if pose.poseCnt > 0 and imu.gyroUpdCnt > 0:
            if hasattr(imu, 'euler') and imu.accUpdCnt > 0:
                self.imu_heading_offset = imu.euler[2] - pose.pose[2]
            else:
                self.imu_heading_offset = self.integrated_heading - pose.pose[2]
    
    def stop(self):
        self.running = False
