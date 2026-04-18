import time
import board
import pwmio
from adafruit_motor import motor
from rotaryio import IncrementalEncoder
import adafruit_hcsr04

# =========================================================
# MOJEEBOT V1 PIN MAP
# =========================================================

# Motors
# M1 = Right motor
RIGHT_MOTOR_A = board.GP8
RIGHT_MOTOR_B = board.GP9

# M2 = Left motor
LEFT_MOTOR_A = board.GP10
LEFT_MOTOR_B = board.GP11

# Encoders
# Right encoder = Grove 6
RIGHT_ENC_A = board.GP26
RIGHT_ENC_B = board.GP27

# Left encoder = Grove 2
LEFT_ENC_A = board.GP2
LEFT_ENC_B = board.GP3

# Ultrasonic = Grove 1
US_TRIG = board.GP0
US_ECHO = board.GP1

# =========================================================
# DIRECTION SETTINGS
# =========================================================
LEFT_DIR = 1
RIGHT_DIR = -1

LEFT_ENC_DIR = -1
RIGHT_ENC_DIR = 1

# =========================================================
# TUNING
# =========================================================
BASE_SPEED = 0.50
REVERSE_SPEED = 0.40
TURN_SPEED = 0.40

LEFT_BIAS = 1.00
RIGHT_BIAS = 1.00

KP = 0.0025
MAX_TRIM = 0.10

STOP_DISTANCE_CM = 20.0
RESUME_DISTANCE_CM = 28.0

BACKUP_TICKS = 200
TURN_TICKS = 320

POST_TURN_DRIVE_TIME = 1.20
PRINT_INTERVAL = 0.25
LOOP_DELAY = 0.02

# =========================================================
# HARDWARE SETUP
# =========================================================
rm_a = pwmio.PWMOut(RIGHT_MOTOR_A, frequency=10000)
rm_b = pwmio.PWMOut(RIGHT_MOTOR_B, frequency=10000)
lm_a = pwmio.PWMOut(LEFT_MOTOR_A, frequency=10000)
lm_b = pwmio.PWMOut(LEFT_MOTOR_B, frequency=10000)

right_motor = motor.DCMotor(rm_a, rm_b)
left_motor = motor.DCMotor(lm_a, lm_b)

right_encoder = IncrementalEncoder(RIGHT_ENC_A, RIGHT_ENC_B)
left_encoder = IncrementalEncoder(LEFT_ENC_A, LEFT_ENC_B)

sonar = adafruit_hcsr04.HCSR04(trigger_pin=US_TRIG, echo_pin=US_ECHO)

# =========================================================
# HELPER FUNCTIONS
# =========================================================
def left_ticks():
    return left_encoder.position * LEFT_ENC_DIR

def right_ticks():
    return right_encoder.position * RIGHT_ENC_DIR

def stop():
    left_motor.throttle = 0
    right_motor.throttle = 0

def set_drive(left, right):
    left_cmd = max(-1.0, min(1.0, left * LEFT_BIAS * LEFT_DIR))
    right_cmd = max(-1.0, min(1.0, right * RIGHT_BIAS * RIGHT_DIR))
    left_motor.throttle = left_cmd
    right_motor.throttle = right_cmd

def read_distance_cm():
    try:
        return sonar.distance
    except RuntimeError:
        return None

def reset_baseline():
    return left_ticks(), right_ticks()

def drive_straight(base_speed, start_left, start_right):
    dl = left_ticks() - start_left
    dr = right_ticks() - start_right

    error = dl - dr
    trim = error * KP

    if trim > MAX_TRIM:
        trim = MAX_TRIM
    elif trim < -MAX_TRIM:
        trim = -MAX_TRIM

    left_speed = base_speed - trim
    right_speed = base_speed + trim

    set_drive(left_speed, right_speed)
    return error, trim, left_speed, right_speed

def backup_ticks(target_ticks, speed):
    start_left, start_right = reset_baseline()
    set_drive(-speed, -speed)

    while True:
        dl = abs(left_ticks() - start_left)
        dr = abs(right_ticks() - start_right)
        avg_ticks = (dl + dr) / 2

        if avg_ticks >= target_ticks:
            break

        time.sleep(0.005)

    stop()

def turn_right_ticks(target_ticks, speed):
    start_left, start_right = reset_baseline()

    # Right turn = left forward, right backward
    set_drive(speed, -speed)

    while True:
        dl = abs(left_ticks() - start_left)
        dr = abs(right_ticks() - start_right)
        avg_ticks = (dl + dr) / 2

        if avg_ticks >= target_ticks:
            break

        time.sleep(0.005)

    stop()

# =========================================================
# MAIN
# =========================================================
print("MojeeBot V1 autonomous avoid mode")
print("Drive straight -> stop -> reverse -> turn right -> continue")
print("Obstacle hysteresis enabled:")
print("  STOP at <=", STOP_DISTANCE_CM, "cm")
print("  RESUME at >=", RESUME_DISTANCE_CM, "cm")

time.sleep(1.0)
stop()

drive_start_left, drive_start_right = reset_baseline()
last_print = time.monotonic()
ignore_obstacles_until = 0
obstacle_active = False

while True:
    now = time.monotonic()
    distance = read_distance_cm()

    # Update obstacle latch using hysteresis
    if distance is not None:
        if not obstacle_active and distance <= STOP_DISTANCE_CM:
            obstacle_active = True
        elif obstacle_active and distance >= RESUME_DISTANCE_CM:
            obstacle_active = False

    # Only trigger a new avoidance event when:
    # 1) obstacle is active
    # 2) we are not in the post-turn ignore window
    if obstacle_active and now >= ignore_obstacles_until:
        print("Obstacle detected:", distance, "cm")
        stop()
        time.sleep(0.10)

        print("Backing up...")
        backup_ticks(BACKUP_TICKS, REVERSE_SPEED)
        time.sleep(0.08)

        print("Turning right...")
        turn_right_ticks(TURN_TICKS, TURN_SPEED)
        time.sleep(0.08)

        # Reset straight-drive baseline after maneuver
        drive_start_left, drive_start_right = reset_baseline()

        # Ignore sonar briefly so it can move away from the same object
        ignore_obstacles_until = time.monotonic() + POST_TURN_DRIVE_TIME

    # Straight driving
    error, trim, left_speed, right_speed = drive_straight(
        BASE_SPEED,
        drive_start_left,
        drive_start_right
    )

    if now - last_print >= PRINT_INTERVAL:
        print(
            "Dist:", distance,
            "ObstacleActive:", obstacle_active,
            "L:", left_ticks(),
            "R:", right_ticks(),
            "Err:", error,
            "Trim:", round(trim, 4),
            "Lspd:", round(left_speed, 3),
            "Rspd:", round(right_speed, 3)
        )
        last_print = now

    time.sleep(LOOP_DELAY)
