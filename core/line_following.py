from mqtt_python.uservice import service
import threading
import time

class LineFollower(threading.Thread):
    def __init__(self):
        super().__init__()
        self.running = True
        self.active = False  # Set to True when you want the robot to drive
        
        # --- PID Tuning Parameters ---
        self.Kp = 0.7
        self.Ki = 0.05
        self.Kd = 0.2
        
        # --- State Variables for PID ---
        self.integral_error = 0.0
        self.last_error = 0.0
        self.last_time = time.time()
        
        # Max build-up for the Integral term to prevent "windup"
        self.max_integral = 2.0 
        
        # --- Sensor Geometry ---
        # Physical weights (distances) of the 8 sensors from the center
        self.sensor_weights = [-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5]
        
        # Control targets
        self.target_speed = 0.0
        self.target_position = 0.0
        
    def run(self):
        """Main thread loop, runs constantly at ~100Hz"""
        while self.running:
            if self.active:
                self._update_control_loop()
            else:
                # Keep last_time updated even when idle so derivative doesn't spike when activated
                self.last_time = time.time() 
                
            time.sleep(0.01)  # ~100 Hz update rate
            
    def _calculate_center_of_mass(self, edge_n):
        """Calculates smooth line position using weighted average"""
        weighted_sum = 0.0
        total_sensor_value = 0.0
        
        for i in range(8):
            # Ignore sensor noise below a certain threshold (e.g., 50)
            val = edge_n[i] if edge_n[i] > 50 else 0
            
            weighted_sum += val * self.sensor_weights[i]
            total_sensor_value += val
            
        # Avoid division by zero if the line is completely lost
        if total_sensor_value == 0:
            print("LineFollower: Warning - Line lost! No valid sensor readings.")
            return None 
            
        return weighted_sum / total_sensor_value

    def _update_control_loop(self):
        """Fetches data, runs PID, and sends motor commands"""
        # Import the global edge sensor data (similar to how Localizer gets pose)
        from mqtt_python.sedge import edge
        
        # 1. Get smooth position
        current_position = self._calculate_center_of_mass(edge.edge_n)
        
        if current_position is None:
            # Line is lost. You could add logic here to spin and search.
            print("LineFollower: Line lost! Stopping robot.")
            return

        print(f"LineFollower: Current Position = {current_position:.2f}")
            
        # 2. Calculate time elapsed (dt)
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0.0:
            dt = 0.001 # Prevent divide-by-zero on extremely fast loops

        print(f"LineFollower: Time since last update = {dt:.3f} seconds")
            
        # 3. Calculate Error
        error = self.target_position - current_position
        
        # 4. Proportional Term
        P_out = self.Kp * error
        
        # 5. Integral Term (with Anti-Windup)
        self.integral_error += error * dt
        # Clamp the integral so it doesn't grow infinitely
        self.integral_error = max(min(self.integral_error, self.max_integral), -self.max_integral)
        I_out = self.Ki * self.integral_error

        print(f"LineFollower: PID Components -> P: {P_out:.3f}, I: {I_out:.3f}, D: (calculated next)")
        
        # 6. Derivative Term
        derivative = (error - self.last_error) / dt
        D_out = self.Kd * derivative
        
        # 7. Compute Total Output
        turn_rate = P_out + I_out + D_out
        
        # Clamp output to safe limits (-4 to 4 rad/s)
        turn_rate = max(min(turn_rate, 4.0), -4.0)
        
        # 8. Send MQTT Command
        # Format: "rc {velocity} {turn_rate} {timestamp}"
        command = f"rc {self.target_speed:.3f} {turn_rate:.3f} {current_time}"
        print(f"LineFollower: Sending command -> {command}")
        service.send("robobot/cmd/ti", command)
        
        # Save state for the next loop iteration
        self.last_error = error
        self.last_time = current_time

    def start_following(self, speed=0.2):
        """Activates the controller and sets forward speed"""
        self.target_speed = speed
        self.integral_error = 0.0 # Reset integral when starting
        self.last_time = time.time()
        self.active = True
        print("LineFollower: Started following line at speed", speed)
        
    def stop_following(self):
        """Pauses the controller without killing the thread"""
        print("LineFollower: Stopped following line")
        self.active = False
        
    def stop(self):
        """Kills the thread completely (call when shutting down the robot)"""
        self.running = False