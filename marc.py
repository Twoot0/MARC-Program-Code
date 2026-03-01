import pwmio
from adafruit_motor import motor
from adafruit_motor import servo
import time
import sys
import math


def seconds_since_boot():
    return f"{time.monotonic():.3f}"


class DriveMotor:
    def __init__(self, forward_pin, reverse_pin, name):
        # Save pin references
        self.forward_pin = forward_pin
        self.reverse_pin = reverse_pin

        # Create PWM outputs for motor control
        self.forward_pwm = pwmio.PWMOut(forward_pin, frequency=1000)
        self.reverse_pwm = pwmio.PWMOut(reverse_pin, frequency=1000)

        # Create the DC motor object using the PWM outputs
        self.motor = motor.DCMotor(self.forward_pwm, self.reverse_pwm)

        self.name = name
        self.untested = True

    # --- Cleaner throttle interface ---
    @property
    def throttle(self):
        return self.motor.throttle

    @throttle.setter
    def throttle(self, value):
        self.motor.throttle = value
    # ----------------------------------

    def test(self, marc=None):
        """
        Interactive test for this motor.
        Press:
        - 'f' → run forward
        - 'r' → run reverse
        - 's' → stop
        - 'q' → quit test
        """
        print(seconds_since_boot() + f" - Starting test for {self.name} motor.")
        print("Commands: 'f' = forward, 'r' = reverse, 's' = stop, 'q' = quit.\n")

        while True:
            command = sys.stdin.read(1).strip().lower()

            if command == 'f':
                self.throttle = 1.0
                print(seconds_since_boot() + f" - {self.name} motor forward (full speed).")

            elif command == 'r':
                self.throttle = -1.0
                print(seconds_since_boot() + f" - {self.name} motor reverse (full speed).")

            elif command == 's':
                self.throttle = 0
                print(seconds_since_boot() + f" - {self.name} motor stopped.")

            elif command == 'q':
                self.throttle = 0
                print(seconds_since_boot() + f" - Exiting {self.name} motor test.")
                break

            else:
                print("Invalid input. Use 'f', 'r', 's', or 'q'.")

            # Small delay to avoid spamming input reads
            time.sleep(0.05)


class ServoMotor:
    def __init__(self, pin, name, min_pulse=500, max_pulse=2500, actuation_range=180, start_angle=0):
        self.pwm = pwmio.PWMOut(pin, duty_cycle=2 ** 15, frequency=50)
        self.servo = servo.Servo(self.pwm, min_pulse=min_pulse, max_pulse=max_pulse, actuation_range=actuation_range)
        self.name = name
        self.start_angle = start_angle
        self.untested = True

class MARC:
    def __init__(self, r_pins, l_pins, x_pins, servo_pins):
        # Initialize motors with names

        self.right = DriveMotor(r_pins[0], r_pins[1], "Right")
        self.left = DriveMotor(l_pins[0], l_pins[1], "Left")
        self.extra = DriveMotor(x_pins[0], x_pins[1], "Extra")

        self.rover_motors = [self.right, self.left, self.extra]
        self.servo0 = ServoMotor(servo_pins[0], "0", min_pulse=500, max_pulse=2500, actuation_range=180, start_angle=90)
        self.servo1 = ServoMotor(servo_pins[1], "1", min_pulse=500, max_pulse=2500, actuation_range=180, start_angle=0)
        self.servo2 = ServoMotor(servo_pins[2], "2", min_pulse=500, max_pulse=2500, actuation_range=180, start_angle=60)
        self.servo3 = ServoMotor(servo_pins[3], "3", min_pulse=500, max_pulse=2500, actuation_range=180, start_angle=90)

        self.servo_memory = [90, 90, 90, 90]

    def check_servo_constraints(self, servo0_angle, servo1_angle, servo2_angle, servo3_angle):
        # Absolute bounds
        if not (0 <= servo0_angle <= 180):
            print(f"⚠️ Servo 0 angle out of bounds: {math.ceil(servo0_angle)}° (must be 0–180°)")
            return False
        if not (0 <= servo1_angle <= 180):
            print(f"⚠️ Servo 1 angle out of bounds: {math.ceil(servo1_angle)}° (must be 0–180°)")
            return False
        if not (0 <= servo2_angle <= 180):
            print(f"⚠️ Servo 2 angle out of bounds: {math.ceil(servo2_angle)}° (must be 0–180°)")
            return False
        if not (0 <= servo3_angle <= 180):
            print(f"⚠️ Servo 3 angle out of bounds: {math.ceil(servo3_angle)}° (must be 0–180°)")
            return False
        # All checks passed
        return True

    def move_single_servo(self, servo_name, angle):
        # 1. Convert "0", "1", etc. to an integer index
        idx = int(servo_name)

        # 2. Create a "test" state based on your MEMORY, not the hardware
        test_state = list(self.servo_memory)
        test_state[idx] = angle # Update only the one we want to move

        # 3. Check constraints using your clean memory values
        if self.check_servo_constraints(*test_state):
            print(f"{seconds_since_boot()} - Moving Servo {servo_name} to {angle}°")

            # Move the physical hardware
            target_servos = [self.servo0, self.servo1, self.servo2, self.servo3]
            target_servos[idx].servo.angle = angle

            # IMPORTANT: Update the memory so the NEXT call knows where we are
            self.servo_memory[idx] = angle
        else:
            print(f"{seconds_since_boot()} - Move to {angle}° BLOCKED. Current Memory: {self.servo_memory}")


    def move_multiple_servos(self, servo0_angle, servo1_angle, servo2_angle, servo3_angle):
        checked = self.check_servo_constraints(servo0_angle, servo1_angle, servo2_angle, servo3_angle)
        if checked:
            # Constraints are satisfied, moving servos
            print(seconds_since_boot() + " - Moving Servos... 0: " + str(servo0_angle) + "°, 1: " + str(servo1_angle) + "°, 2: " + str(servo2_angle) + "°, 3: " + str(servo3_angle))
            self.servo0.servo.angle = servo0_angle
            self.servo1.servo.angle = servo1_angle
            self.servo2.servo.angle = servo2_angle
            self.servo3.servo.angle = servo3_angle

    # --- Utility Methods ---
    def _validate_throttles(self, throttles):
        for t in throttles:
            if t is not None and (t < -1.0 or t > 1.0):
                raise ValueError("Motor throttles must be between -1.0 and +1.0 or None")

    # --- Motor Control Methods ---
    def set_multiple_motor_throttles_timed(self, motor_throttles, runtime):
        r_target, l_target, x_target = motor_throttles
        self._validate_throttles(motor_throttles)

        print(
            seconds_since_boot()
            + f" - Set motor throttles: R: {r_target} | L: {l_target} | Xtra: {x_target}, run for {runtime} seconds."
        )

        self.right.throttle = r_target
        self.left.throttle = l_target
        self.extra.throttle = x_target

        time.sleep(runtime)

        self.right.throttle = 0
        self.left.throttle = 0
        self.extra.throttle = 0

        print(seconds_since_boot() + " - Stopping all motors.")

    def set_multiple_motor_throttles(self, motor_throttles, runtime):
        r_target, l_target, x_target = motor_throttles
        self._validate_throttles(motor_throttles)

        print(
            seconds_since_boot()
            + f" - Set motor throttles: R: {r_target} | L: {l_target} | Xtra: {x_target}, runtime placeholder {runtime}."
        )

        self.right.throttle = r_target
        self.left.throttle = l_target
        self.extra.throttle = x_target

    def set_motor_throttle_timed(self, motor, throttle, runtime):
        if throttle is not None and (throttle < -1.0 or throttle > 1.0):
            raise ValueError("Motor throttle must be between -1.0 and +1.0 or None")

        print(seconds_since_boot() + f" - Setting {motor.name} motor to throttle {throttle}")
        motor.throttle = throttle
        time.sleep(runtime)
        print(seconds_since_boot() + f" - Stopping {motor.name} motor.")
        motor.throttle = 0

    def set_motor_throttle(self, motor, throttle):
        if throttle is not None and (throttle < -1.0 or throttle > 1.0):
            raise ValueError("Motor throttle must be between -1.0 and +1.0 or None")

        print(seconds_since_boot() + f" - Setting {motor.name} motor to throttle {throttle}")
        motor.throttle = throttle

    def wait(self, seconds):
        print(seconds_since_boot() + f" - Waiting for {seconds} seconds...")
        time.sleep(seconds)
        print("Waiting done")

    def test(self):
            """
            Allows testing each motor one by one interactively.
            """
            print(seconds_since_boot() + " - Entering full test mode.")
            print("Type '1' to test Right motor, '2' for Left motor, '3' for Extra motor, or 'q' to quit.\n")

            while True:
                command = sys.stdin.read(1).strip().lower()

                if command == '1':
                    self.right.test()
                elif command == '2':
                    self.left.test()
                elif command == '3':
                    self.extra.test()
                elif command == 'q':
                    print(seconds_since_boot() + " - Exiting full test mode.")
                    break
                else:
                    print("Invalid input. Use '1', '2', '3', or 'q'.")

                time.sleep(0.05)
