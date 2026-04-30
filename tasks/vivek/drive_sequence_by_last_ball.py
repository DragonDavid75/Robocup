import math

from tasks.base_task import BaseTask, TaskStatus


class DriveSequenceByLastBallTask(BaseTask):
    """
    Runs a different drive sequence based on world.first_ball.

    Expected:
        "red"
        "blue"
        anything else / None -> fallback sequence
    """

    def __init__(self, world, motion_controller, servo_controller):
        super().__init__(world, motion_controller, servo_controller)

        self.state = 0
        self.sequence = []
        self.sequence_index = 0
        self.last_ball = None

        self.blue_sequence = [
            ("turn", -10),
            ("drive_to_line", 0.2),
            ("turn", -60),
        ]

        self.red_sequence = [
            ("turn", 180),
            ("drive_to_line", 0.15),
            ("turn", -90),
        ]

        # Used if neither red nor blue was found
        self.none_sequence = [
            ("turn", -10),
            ("drive_to_line", 0.2),
            ("turn", -60),
        ]

    def start(self):
        super().start()

        self.state = 0
        self.sequence_index = 0
        self.sequence = []

        self.last_ball = getattr(self.world, "first_ball", None)

        print("[TASK] DriveSequenceByLastBallTask started")
        print(f"[TASK] Last ball from world.first_ball = {self.last_ball}")

        if self.last_ball == "blue":
            self.sequence = self.blue_sequence
            print("[TASK] Selected BLUE drive sequence")

        elif self.last_ball == "red":
            self.sequence = self.red_sequence
            print("[TASK] Selected RED drive sequence")

        else:
            self.sequence = self.none_sequence
            print("[TASK] No red/blue ball found. Selected fallback sequence.")

    def update(self):
        if self.state == 0:
            if not self.sequence:
                self.state = 99
            else:
                self.state = 1

        elif self.state == 1:
            if self.sequence_index >= len(self.sequence):
                self.state = 99
                return TaskStatus.RUNNING

            step = self.sequence[self.sequence_index]
            command = step[0]

            if command == "drive":
                _, distance_m, speed = step
                print(
                    f"[TASK] Step {self.sequence_index + 1}/{len(self.sequence)}: "
                    f"drive {distance_m:.2f} m at speed {speed:.2f}"
                )
                self.motion_controller.drive_distance(distance_m, speed)
                self.state = 2

            elif command == "turn":
                _, angle_deg = step
                print(
                    f"[TASK] Step {self.sequence_index + 1}/{len(self.sequence)}: "
                    f"turn {angle_deg:.2f} deg"
                )
                self.motion_controller.turn_in_place(math.radians(angle_deg))
                self.state = 2

            elif command == "drive_to_line":
                _, speed = step
                print(
                    f"[TASK] Step {self.sequence_index + 1}/{len(self.sequence)}: "
                    f"drive_to_line at speed {speed:.2f}"
                )
                self.motion_controller.drive_to_line(speed)
                self.state = 2

            elif command == "follow_until_intersection_or_end_line":
                _, speed = step
                print(
                    f"[TASK] Step {self.sequence_index + 1}/{len(self.sequence)}: "
                    f"follow_until_intersection_or_end_line at speed {speed:.2f}"
                )
                self.motion_controller.follow_until_intersection_or_end_line(speed)
                self.state = 2

            elif command == "follow_for_distance":
                _, distance_m, speed, action = step
                print(
                    f"[TASK] Step {self.sequence_index + 1}/{len(self.sequence)}: "
                    f"follow_for_distance {distance_m:.2f} m "
                    f"at speed {speed:.2f}, action={action}"
                )
                self.motion_controller.follow_for_distance(
                    distance_m,
                    speed,
                    action=action,
                )
                self.state = 2

            else:
                print(f"[TASK] Unknown sequence command: {command}")
                self.state = 99

        elif self.state == 2:
            if not self.motion_controller.is_busy():
                self.sequence_index += 1
                self.state = 1

        elif self.state == 99:
            if not self.motion_controller.is_busy():
                print("[TASK] DriveSequenceByLastBallTask completed")
                return TaskStatus.DONE

        return TaskStatus.RUNNING

    def stop(self):
        super().stop()
        self.motion_controller.stop()
        print("[TASK] DriveSequenceByLastBallTask stopped")