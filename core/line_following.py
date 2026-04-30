from mqtt_python.uservice import service
from mqtt_python.sedge import edge
import threading
import time

class LineFollower(threading.Thread):
    def __init__(self):
        super().__init__()
        self.running = True
        self.active = False  # Set to True when you want the robot to drive
        
        # --- PID Tuning Parameters ---
        self.Kp = 0.9
        self.Ki = 0
        self.Kd = 0.5
        
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
        self.turn_speed = 0.1
        self.last_crossing_time = 0.0
        self.turn_cooldown = 0.6
        
        # --- Turn Intention ---
        self.action = "STRAIGHT" # Options: "STRAIGHT", "LEFT", "RIGHT", "STOP"
        
    def run(self):
        """Main thread loop, runs constantly at ~100Hz"""
        while self.running:
            if self.active:
                self._update_control_loop()
            else:
                # Keep last_time updated even when idle so derivative doesn't spike when activated
                self.last_time = time.time() 
                
            time.sleep(0.01)  # ~100 Hz update rate
            
    def _calculate_center_of_mass(self, edge_n, in_turn_mode):
        """Calculates smooth line position using weighted average and masking"""
        weighted_sum = 0.0
        total_sensor_value = 0.0
        
        for i in range(8):
            # --- SENSOR MASKING LOGIC ---
            # Use in_turn_mode instead of edge.crossingLine
            if in_turn_mode and self.action == "LEFT" and i >= 4:
                continue
            if in_turn_mode and self.action == "RIGHT" and i <= 3:
                continue

            val = edge_n[i] if edge_n[i] > 50 else 0
            weighted_sum += val * self.sensor_weights[i]
            total_sensor_value += val
            
        if total_sensor_value == 0:
            print("LineFollower: Warning - Line lost! No valid sensor readings.")
            return None 
            
        return weighted_sum / total_sensor_value

    def _update_control_loop(self):
        """Fetches data, runs PID, and sends motor commands"""
        from mqtt_python.sedge import edge
        
        # --- NEW: Get time first and check the cooldown timer ---
        current_time = time.time()
        
        if edge.crossingLine:
            self.last_crossing_time = current_time
            
        # This will stay True for 0.6s after the crossing line is gone
        in_turn_mode = (current_time - self.last_crossing_time) < self.turn_cooldown
        
        # 1. Get smooth position (pass the timer state)
        current_position = self._calculate_center_of_mass(edge.edge_n, in_turn_mode)
        
        if current_position is None:
            print("LineFollower: Line lost! Stopping robot.")
            service.send("robobot/cmd/ti", f"rc 0.0 0.0 {time.time()}")
            return

        # 2. Calculate time elapsed (dt)
        dt = current_time - self.last_time
        if dt <= 0.0:
            dt = 0.001
            
        # 3. Calculate Error
        error = self.target_position - current_position
        
        # 4. Proportional Term
        P_out = self.Kp * error
        
        # 5. Integral Term
        self.integral_error += error * dt
        self.integral_error = max(min(self.integral_error, self.max_integral), -self.max_integral)
        I_out = self.Ki * self.integral_error
        
        # 6. Derivative Term
        derivative = (error - self.last_error) / dt
        D_out = self.Kd * derivative
        
        # 7. Compute Total Output
        turn_rate = max(min(P_out + I_out + D_out, 4.0), -4.0)
        
        # 8. Send MQTT Command (using the timer state instead of raw crossingLine)
        # if in_turn_mode:
        #     command = f"rc {self.turn_speed:.3f} {turn_rate:.3f} {current_time}"
        # else:
        #     command = f"rc {self.target_speed:.3f} {turn_rate:.3f} {current_time}"
        # service.send("robobot/cmd/ti", command)

        # --- 7.5 Variable Velocity Calculation ---
        abs_error = abs(error)
        max_possible_error = 3.5  # Max weight of your sensors
        
        # Define your dynamic range based on the target_speed
        min_v = self.target_speed - 0.1
        max_v = self.target_speed + 0.1
        
        # Calculate how much to "slow down" from the maximum possible speed
        # When error is 0, we want max_v. When error is max, we want min_v.
        total_range = max_v - min_v
        adaptive_speed = max_v - (total_range * (abs_error / max_possible_error))
        
        # Ensure the speed is strictly clamped between your desired offsets
        adaptive_speed = max(min_v, min(max_v, adaptive_speed))
        
        # --- 8. Send MQTT Command ---
        if in_turn_mode:
            # Use the minimum stable speed during detected intersections/turns
            command = f"rc {self.turn_speed:.3f} {turn_rate:.3f} {current_time}"
        else:
            # Use the new calculated adaptive speed for normal following
            command = f"rc {adaptive_speed:.3f} {turn_rate:.3f} {current_time}"
        
        service.send("robobot/cmd/ti", command)
        
        # Save state for the next loop iteration
        self.last_error = error
        self.last_time = current_time

    """
    Call this method to start following the line with a specific speed and turn behavior at intersections.
    Speed: The forward speed to maintain while following the line (in meters per second).
    Action: The behavior to execute at intersections. Options are "STRAIGHT", "LEFT", "RIGHT". Default is "STRAIGHT".
    """
    def start_following(self, speed=0.2, action="STRAIGHT"):
        """
        Activates the controller and sets forward speed and intersection behavior.
        action options: "STRAIGHT", "LEFT", "RIGHT"
        """
        self.target_speed = speed
        self.action = action.upper()
        self.integral_error = 0.0 # Reset integral when starting
        self.last_time = time.time()
        self.active = True
        print(f"LineFollower: Started at speed {speed}, Mode: {self.action}")
        
    """
    Call this method to stop following the line immediately.
    """
    def stop_following(self):
        """Pauses the controller and commands motors to halt"""
        print("LineFollower: Stopped following line")
        self.active = False
        service.send("robobot/cmd/ti", f"rc 0.0 0.0 {time.time()}")
        
    def stop(self):
        """Kills the thread completely"""
        self.running = False
