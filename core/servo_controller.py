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
        print("% ServoController: Starting servo controller thread...")
        while self.running:
            print(f"% ServoController: Current task: {self.current_task}")
            if self.current_task == 'servo_control':
                print("% ServoController: Running servo control task...")
                self._handle_servo_control()
            elif service.stop:
                print("% ServoController: Emergency stop activated!")
                self.stop()
            
            time.sleep(0.05)  # 20 Hz loop rate

    def _handle_servo_control(self):
        """Monitors the timer for the servo motion."""
        start_time = self.task_params.get("start_time", 0)
        duration = self.task_params.get("duration", 0)

        print(f"% ServoController: Monitoring servo control task. Elapsed time: {time.time() - start_time:.2f}s / {duration:.2f}s")
        
        # Once the time is up, just clear the task. No hardware commands sent!
        if time.time() - start_time >= duration:
            print("% ServoController: Servo wait time complete.")
            self.current_task = None

    # --- High Level Commands ---

    """
    Moves a servo to a specified position at a given speed, and starts a timer to monitor the duration of the motion.
    Idx: The index of the servo to control.
    Pos: The target position for the servo.
    Speed: The speed at which to move the servo.
    Duration: Time to wait after sending the command before allowing another command. This is because we don't get feedback from the servo, so we use a timer to estimate when the motion should be complete. Default is 0.5 seconds.
    """
    def servo_control(self, idx, pos, speed, duration=0.5):
        print(f"% ServoController: Moving servo {idx} to {pos}. Waiting {duration}s...")
        
        # 1. Send the hardware command immediately (Safe because it's the main thread)
        self.robot.set_servo(idx, pos, speed)
        
        # 2. Setup the timer parameters for the background thread to monitor
        self.task_params["duration"] = duration
        self.task_params["start_time"] = time.time()
        self.current_task = 'servo_control'

    """
    Call this method to check if the servo controller is currently executing a task.
    Returns True if a task is in progress, False otherwise.
    Use it to prevent starting a new task while another one is still running.
    """
    def is_busy(self):
        """Returns True if the servo timer is currently running."""
        return self.current_task is not None

    def stop(self):
        """Safely stops the servo controller thread."""
        self.running = False