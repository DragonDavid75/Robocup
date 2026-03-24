# core/servo_controller.py
import threading
import time
from mqtt_python.uservice import service

class ServoController(threading.Thread):
    def __init__(self, robot):
        super().__init__()
        self.robot = robot
        self.running = True
        self.current_task = None
        self.task_params = {}

    def run(self):
        while self.running:
            if self.current_task == 'servo_control':
                self._handle_servo_control()
            elif service.stop:
                print("% ServoController: Emergency stop activated!")
                self.stop()
            
            time.sleep(0.05)  # 20 Hz loop rate

    def _handle_servo_control(self):
        """Monitors the timer for the servo motion."""
        start_time = self.task_params.get("start_time", 0)
        duration = self.task_params.get("duration", 0)
        
        # Once the time is up, just clear the task. No hardware commands sent!
        if time.time() - start_time >= duration:
            print("% ServoController: Servo wait time complete.")
            self.current_task = None

    # --- High Level Commands ---

    def servo_control(self, idx, pos, speed, duration=0):
        """
        Sends the command immediately from the main thread and 
        tells the background thread to start the wait timer.
        """
        print(f"% ServoController: Moving servo {idx} to {pos}. Waiting {duration}s...")
        
        # 1. Send the hardware command immediately (Safe because it's the main thread)
        self.robot.set_servo(idx, pos, speed)
        
        # 2. Setup the timer parameters for the background thread to monitor
        self.task_params["duration"] = duration
        self.task_params["start_time"] = time.time()
        self.current_task = 'servo_control'

    def is_busy(self):
        """Returns True if the servo timer is currently running."""
        return self.current_task is not None

    def stop(self):
        """Safely stops the servo controller thread."""
        self.running = False