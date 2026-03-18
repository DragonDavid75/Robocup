# core/localizer.py
import threading
import time
import numpy as np

# Set USE_LOCALIZER = True to enable full localization with IMU correction
# Set to False for simple pose bridge (just copy odometry to WorldModel)
USE_LOCALIZER = True

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
        
        # IMU-based distance tracking (accelerometer)
        self.last_acc_time = None
        self.imu_vel_x = 0.0  # integrated velocity in world frame
        self.imu_vel_y = 0.0
        self.imu_pos_x = 0.0  # double-integrated position
        self.imu_pos_y = 0.0
        self.acc_offset = [0.0, 0.0, 0.0]  # accelerometer bias (gravity + offset)
        self.acc_calibrated = False
        
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
        
        if pose.poseCnt > 0:
            self.world.set_pose(pose.pose[0], pose.pose[1], pose.pose[2])
        
        if imu.gyroUpdCnt > 0:
            self.world.set_imu(self.integrated_heading, imu.gyro[2])
    
    def _update_localized_pose(self):
        """Dead reckoning with IMU heading correction"""
        from mqtt_python.spose import pose
        from mqtt_python.simu import imu
        
        # Get odometry from Teensy
        if pose.poseCnt == 0:
            return
        
        odom_x = pose.pose[0]
        odom_y = pose.pose[1]
        odom_h = pose.pose[2]
        
        # Integrate gyro for heading (if IMU available)
        if imu.gyroUpdCnt > 0:
            gyro_z = imu.gyro[2]
            gyro_x = imu.gyro[0]
            gyro_y = imu.gyro[1]
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
            
            self.world.set_imu(np.deg2rad(imu_heading), gyro_z)
        else:
            corrected_h = odom_h
        
        # IMU-based distance calculation (accelerometer double integration)
        if imu.accUpdCnt > 0:
            now = time.time()
            
            # Calibrate accelerometer offset on first readings (assume robot stationary)
            if not self.acc_calibrated and pose.poseCnt > 5:
                self.acc_offset = [imu.acc[0], imu.acc[1], imu.acc[2]]
                self.acc_calibrated = True
            
            if self.acc_calibrated and self.last_acc_time is not None:
                dt = now - self.last_acc_time
                
                # Remove offset (gravity component) and get linear acceleration
                acc_x = imu.acc[0] - self.acc_offset[0]
                acc_y = imu.acc[1] - self.acc_offset[1]
                
                # Rotate to world frame using heading
                acc_world_x = acc_x * np.cos(corrected_h) - acc_y * np.sin(corrected_h)
                acc_world_y = acc_x * np.sin(corrected_h) + acc_y * np.cos(corrected_h)
                
                # Integrate acceleration to get velocity
                self.imu_vel_x += acc_world_x * dt
                self.imu_vel_y += acc_world_y * dt
                
                # Integrate velocity to get position
                self.imu_pos_x += self.imu_vel_x * dt
                self.imu_pos_y += self.imu_vel_y * dt
            
            self.last_acc_time = now
        
        # Apply map offset (transform from odometry to map coordinates)
        map_x = odom_x + MAP_OFFSET_X
        map_y = odom_y + MAP_OFFSET_Y
        
        self.world.set_pose(map_x, map_y, corrected_h)
    
    def reset_pose(self, x=0.0, y=0.0, h=0.0):
        """Reset localization to a known position"""
        self.last_odom_x = x
        self.last_odom_y = y
        self.last_odom_h = h
        self.imu_heading_offset = 0.0
        self.integrated_heading = 0.0
        self.imu_vel_x = 0.0
        self.imu_vel_y = 0.0
        self.imu_pos_x = 0.0
        self.imu_pos_y = 0.0
        self.acc_calibrated = False
    
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
