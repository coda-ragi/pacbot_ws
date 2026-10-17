
#!/usr/bin/env python3

'''
# Team ID:          <5224>
# Theme:            PacBot
# Author List:      <Anurag Singh, Ashutosh Vishwakarma, Saket Kr. Rai, Rana Pratap Singh>
# Filename:         task_1b_controller.py
# Functions:        _mqtt_client, clamp, sanitize_distance, get_sensor_values,
#                   get_front_distance, calculate_available_distance,
#                   left_is_open, right_is_open, reset_pid, reset_segment,
#                   reset_turn, send_velocity, stop_robot,
#                   calculate_pid_correction, follow_corridor,
#                   calculate_drive_speed, choose_turn_direction, start_turn,
#                   turn_is_aligned, perform_turn, perform_entry,
#                   perform_calculate, perform_fast_move, perform_approach,
#                   perform_wait_for_opening, perform_straighten, perform_clearance,
#                   on_message, on_connect, main
# Global variables: MQTT_HOST, MQTT_PORT, TOPIC_SENSORS, TOPIC_WHEEL_VEL,
#                   BASE_SPEED, APPROACH_SPEED, TURN_OUTER_SPEED,
#                   TURN_INNER_SPEED, TURN_START_OUTER_SPEED,
#                   TURN_START_INNER_SPEED, STRAIGHTEN_SPEED, MAX_SPEED,
#                   NO_HIT_DISTANCE, TARGET_SIDE_DISTANCE, SIDE_OPEN_DISTANCE, JUNCTION_OPENING_DELTA,
#                   FRONT_FAST_DISTANCE, FRONT_APPROACH_DISTANCE,
#                   FRONT_TURN_DISTANCE, TURN_CLEAR_DISTANCE, TURN_MAX_TIME, TURN_TARGET_ANGLE, TURN_ANGLE_TOLERANCE, TURN_BASE_SPEED, TURN_ANGLE_KP, TURN_MIN_SPEED, CLEARANCE_DISTANCE, CLEARANCE_SPEED, WHEEL_RADIUS_M, SAFETY_MARGIN,
#                   ALIGNMENT_TOLERANCE, TURN_CONFIRM_TIME, TURN_MIN_TIME, TURN_FRONT_GAIN, JUNCTION_CONFIRM_TIME, TURN_MAX_TIME, STRAIGHTEN_TIME,
#                   ENTRY_TIME, KP, KI, KD, INTEGRAL_LIMIT, state, entry_timer, opening_wait_timer, diagnostic_timer,
#                   segment_distance, available_distance, previous_front_distance,
#                   integral, previous_error, turn_direction, turn_timer,
#                   turn_clear_timer, straighten_timer, turn_angle, clearance_distance
'''

'''
Purpose:
---
PacBot Task 1B controller for sensor-based maze navigation.

The controller is organised around movement phases instead of fixed-angle
rotations. PacBot moves quickly through a clear corridor, calculates the
remaining usable distance, slows while approaching a decision point, makes
a smooth curved turn, straightens, and then calculates the next segment.

Sensor configuration:
    FL -> left-facing sensor
    FR -> right-facing sensor
    SL -> forward-facing sensor
    SR -> forward-facing sensor

MQTT topics:
    Subscribe: pacbot/sensors
    Publish:   pacbot/wheel_vel
'''

import json
import math

import paho.mqtt.client as mqtt


# MQTT configuration used by the simulator.
MQTT_HOST = "localhost"
MQTT_PORT = 1883

TOPIC_SENSORS = "pacbot/sensors"
TOPIC_WHEEL_VEL = "pacbot/wheel_vel"


def _mqtt_client():
    '''
    Purpose:
    ---
    Create an MQTT client compatible with the installed Paho MQTT version.

    Input Arguments:
    ---
    None.

    Returns:
    ---
    `client` : [ mqtt.Client ]
        Configured MQTT client object.

    Example call:
    ---
    client = _mqtt_client()
    '''

    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:
        return mqtt.Client()


# Main straight-line wheel command.
BASE_SPEED = 7.5

# Speed used while approaching a wall or junction.
APPROACH_SPEED = 5.0

# Outer-wheel speed during a smooth turn.
TURN_OUTER_SPEED = 4.5

# Inner-wheel speed during the main part of a smooth turn.
TURN_INNER_SPEED = 1.8

# Lower turn speeds used when the turn is first initiated.
TURN_START_OUTER_SPEED = 3.5
TURN_START_INNER_SPEED = 1.5

# Speed used briefly after a turn to settle the robot into the new corridor.
STRAIGHTEN_SPEED = 4.5

# Absolute wheel-command limit.
MAX_SPEED = 10.0


# A very large sensor value means that the ray did not hit a nearby wall.
NO_HIT_DISTANCE = 5.0

# Desired lateral clearance used when only one side wall is visible.
TARGET_SIDE_DISTANCE = 0.20

# A side ray above this value is treated as an available opening.
SIDE_OPEN_DISTANCE = 0.18

# Above this distance, the robot can travel at normal speed.
FRONT_FAST_DISTANCE = 0.50

# Inside this distance, the robot should already be slowing down.
FRONT_APPROACH_DISTANCE = 0.38

# Inside this distance, the robot prepares to turn.
FRONT_TURN_DISTANCE = 0.28

# A turn is considered clear when the new forward corridor exceeds this value.
TURN_CLEAR_DISTANCE = 0.35

# Extra distance kept between the robot and the front wall.
SAFETY_MARGIN = 0.07

# Maximum side-distance difference accepted while straightening.
ALIGNMENT_TOLERANCE = 0.08

# The front-clear condition must persist before a turn ends.
TURN_CONFIRM_TIME = 0.08
TURN_MIN_TIME = 0.25
TURN_MAX_TIME = 4.00

# Gyro-controlled turn and post-turn travel calibration.
TURN_TARGET_ANGLE = math.pi / 2.0
TURN_ANGLE_TOLERANCE = math.radians(3.0)
TURN_BASE_SPEED = 2.8
TURN_ANGLE_KP = 2.0
TURN_MIN_SPEED = 1.0
CLEARANCE_DISTANCE = 0.18
CLEARANCE_SPEED = 3.5
# Calibrate this radius against measured simulator travel.
WHEEL_RADIUS_M = 0.03

# A junction is detected from a change in a side-facing ray, not from
# comparing two equal wall distances at one instant.
JUNCTION_OPENING_DELTA = 0.05
JUNCTION_CONFIRM_TIME = 0.06

# A turn is complete only after the forward view has opened relative to
# the view recorded when the turn began.
TURN_FRONT_GAIN = 0.10

# Short straightening phase after a curved turn.
STRAIGHTEN_TIME = 0.12

# Time spent moving through the initial entry region.
ENTRY_TIME = 0.25


# PID values for centring between side walls.
KP = 10.0
KI = 0.10
KD = 0.30

# Integral anti-windup limit.
INTEGRAL_LIMIT = 0.50


# Controller states.
STATE_ENTRY = 0
STATE_CALCULATE = 1
STATE_FAST_MOVE = 2
STATE_APPROACH = 3
STATE_WAIT_FOR_OPENING = 4
STATE_TURN_LEFT = 5
STATE_TURN_RIGHT = 6
STATE_STRAIGHTEN = 7
STATE_CLEARANCE = 8
STATE_HALT = 9


# Current controller state.
state = STATE_ENTRY

# Entry-phase timer.
entry_timer = 0.0

# Estimated distance travelled during the current movement phase.
segment_distance = 0.0

# Current sensor-based estimate of free distance ahead.
available_distance = 0.0

# Previous front distance used for diagnostics.
previous_front_distance = NO_HIT_DISTANCE

# PID memory.
integral = 0.0
previous_error = 0.0

# Direction locked for the current turn.
turn_direction = 0

# Time spent in the current turn.
turn_timer = 0.0

# Time for which the front-clear condition has remained true.
turn_clear_timer = 0.0

# Straightening timer.
straighten_timer = 0.0

# Timer used while waiting for a side opening to appear.
opening_wait_timer = 0.0

# Timer used to limit diagnostic output without flooding the terminal.
diagnostic_timer = 0.0

# Side readings recorded when the robot first enters a junction.
junction_left_reference = 0.10
junction_right_reference = 0.10
junction_open_timer = 0.0

# Forward clearance recorded at the beginning of a turn.
turn_start_front_distance = 0.0
turn_angle = 0.0
turn_diagnostic_timer = 0.0
clearance_distance = 0.0


def clamp(value, minimum, maximum):
    '''
    Purpose:
    ---
    Restrict a numeric value to a specified range.

    Input Arguments:
    ---
    `value` : [ float ]
        Value to be restricted.
    `minimum` : [ float ]
        Lower bound of the range.
    `maximum` : [ float ]
        Upper bound of the range.

    Returns:
    ---
    `clamped_value` : [ float ]
        Value limited to the requested range.

    Example call:
    ---
    speed = clamp(speed, -MAX_SPEED, MAX_SPEED)
    '''

    return max(minimum, min(maximum, value))


def sanitize_distance(distance):
    '''
    Purpose:
    ---
    Convert a sensor distance to a safe numeric value.

    Input Arguments:
    ---
    `distance` : [ float ]
        Raw distance reported by a ToF sensor.

    Returns:
    ---
    `distance` : [ float ]
        Non-negative finite distance limited to NO_HIT_DISTANCE.

    Example call:
    ---
    front = sanitize_distance(data["sl"])
    '''

    try:
        distance = float(distance)
    except (TypeError, ValueError):
        return NO_HIT_DISTANCE

    if not math.isfinite(distance) or distance < 0.0:
        return NO_HIT_DISTANCE

    return min(distance, NO_HIT_DISTANCE)


def get_sensor_values(data):
    '''
    Purpose:
    ---
    Extract and sanitise all PacBot sensor values from an MQTT message.

    Input Arguments:
    ---
    `data` : [ dict ]
        Decoded JSON dictionary received from pacbot/sensors.

    Returns:
    ---
    `fl` : [ float ]
        Left-facing sensor distance.
    `fr` : [ float ]
        Right-facing sensor distance.
    `sl` : [ float ]
        Forward-left sensor distance.
    `sr` : [ float ]
        Forward-right sensor distance.
    `yaw_rate` : [ float ]
        Z-axis angular velocity from the gyro.
    `dt` : [ float ]
        Simulation time elapsed since the previous sensor update.

    Example call:
    ---
    fl, fr, sl, sr, yaw_rate, dt = get_sensor_values(data)
    '''

    fl = sanitize_distance(data.get("fl", NO_HIT_DISTANCE))
    fr = sanitize_distance(data.get("fr", NO_HIT_DISTANCE))
    sl = sanitize_distance(data.get("sl", NO_HIT_DISTANCE))
    sr = sanitize_distance(data.get("sr", NO_HIT_DISTANCE))

    try:
        yaw_rate = float(data.get("gyro", [0.0, 0.0, 0.0])[2])
    except (TypeError, IndexError, ValueError):
        yaw_rate = 0.0

    try:
        dt = float(data.get("dt", 0.0))
    except (TypeError, ValueError):
        dt = 0.0

    return fl, fr, sl, sr, yaw_rate, max(0.0, dt)


def get_front_distance(sl, sr):
    '''
    Purpose:
    ---
    Estimate the usable distance in front of PacBot.

    The closer forward-facing ray is used because it is safer for collision
    avoidance than averaging two rays when one ray sees a wall earlier.

    Input Arguments:
    ---
    `sl` : [ float ]
        Forward-left sensor distance.
    `sr` : [ float ]
        Forward-right sensor distance.

    Returns:
    ---
    `front_distance` : [ float ]
        Conservative forward distance estimate.

    Example call:
    ---
    front_distance = get_front_distance(sl, sr)
    '''

    return min(sl, sr)


def calculate_available_distance(front_distance):
    '''
    Purpose:
    ---
    Calculate the distance that can be travelled before the next front
    obstacle while preserving a safety margin.

    Input Arguments:
    ---
    `front_distance` : [ float ]
        Conservative distance to the nearest front obstacle.

    Returns:
    ---
    `available_distance` : [ float ]
        Estimated usable distance in metres after the safety margin.

    Example call:
    ---
    distance = calculate_available_distance(front_distance)
    '''

    if front_distance >= NO_HIT_DISTANCE:
        return NO_HIT_DISTANCE

    return max(0.0, front_distance - SAFETY_MARGIN)


def left_is_open(fl):
    '''
    Purpose:
    ---
    Determine whether the left side currently provides an opening.

    Input Arguments:
    ---
    `fl` : [ float ]
        Left-facing sensor distance.

    Returns:
    ---
    `is_open` : [ bool ]
        True when the left sensor sees enough free space.

    Example call:
    ---
    if left_is_open(fl):
        ...
    '''

    return fl > SIDE_OPEN_DISTANCE


def right_is_open(fr):
    '''
    Purpose:
    ---
    Determine whether the right side currently provides an opening.

    Input Arguments:
    ---
    `fr` : [ float ]
        Right-facing sensor distance.

    Returns:
    ---
    `is_open` : [ bool ]
        True when the right sensor sees enough free space.

    Example call:
    ---
    if right_is_open(fr):
        ...
    '''

    return fr > SIDE_OPEN_DISTANCE


def reset_pid():
    '''
    Purpose:
    ---
    Clear PID memory before entering a new movement segment.

    Input Arguments:
    ---
    None.

    Returns:
    ---
    None.

    Example call:
    ---
    reset_pid()
    '''

    global integral
    global previous_error

    integral = 0.0
    previous_error = 0.0


def reset_segment():
    '''
    Purpose:
    ---
    Clear the travel-distance estimate for a newly calculated corridor.

    Input Arguments:
    ---
    None.

    Returns:
    ---
    None.

    Example call:
    ---
    reset_segment()
    '''

    global segment_distance
    global previous_front_distance

    segment_distance = 0.0
    previous_front_distance = NO_HIT_DISTANCE


def reset_turn():
    '''
    Purpose:
    ---
    Clear all variables associated with the current curved turn.

    Input Arguments:
    ---
    None.

    Returns:
    ---
    None.

    Example call:
    ---
    reset_turn()
    '''

    global turn_direction
    global turn_timer
    global turn_clear_timer
    global turn_angle
    global turn_diagnostic_timer

    turn_direction = 0
    turn_timer = 0.0
    turn_clear_timer = 0.0
    turn_angle = 0.0
    turn_diagnostic_timer = 0.0


def send_velocity(client, left, right):
    '''
    Purpose:
    ---
    Publish a bounded pair of wheel velocity commands to PacBot.

    Input Arguments:
    ---
    `client` : [ mqtt.Client ]
        MQTT client used to publish the command.
    `left` : [ float ]
        Command for the left wheel.
    `right` : [ float ]
        Command for the right wheel.

    Returns:
    ---
    None.

    Example call:
    ---
    send_velocity(client, BASE_SPEED, BASE_SPEED)
    '''

    left = clamp(left, -MAX_SPEED, MAX_SPEED)
    right = clamp(right, -MAX_SPEED, MAX_SPEED)

    client.publish(
        TOPIC_WHEEL_VEL,
        json.dumps({
            "left": float(left),
            "right": float(right)
        })
    )


def stop_robot(client):
    '''
    Purpose:
    ---
    Command both wheels to stop.

    Input Arguments:
    ---
    `client` : [ mqtt.Client ]
        MQTT client used to publish the stop command.

    Returns:
    ---
    None.

    Example call:
    ---
    stop_robot(client)
    '''

    send_velocity(client, 0.0, 0.0)


def calculate_pid_correction(fl, fr, dt):
    '''
    Purpose:
    ---
    Calculate a bounded steering correction from the two side-facing
    distance measurements.

    Input Arguments:
    ---
    `fl` : [ float ]
        Left-facing sensor distance.
    `fr` : [ float ]
        Right-facing sensor distance.
    `dt` : [ float ]
        Simulation time step in seconds.

    Returns:
    ---
    `correction` : [ float ]
        Differential wheel-speed correction.

    Example call:
    ---
    correction = calculate_pid_correction(fl, fr, dt)
    '''

    global integral
    global previous_error

    left_valid = fl < NO_HIT_DISTANCE
    right_valid = fr < NO_HIT_DISTANCE

    if not left_valid and not right_valid:
        reset_pid()
        return 0.0

    if left_valid and right_valid:
        error = fr - fl
    elif left_valid:
        error = TARGET_SIDE_DISTANCE - fl
    else:
        error = fr - TARGET_SIDE_DISTANCE

    integral += error * dt
    integral = clamp(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT)

    if dt > 0.0:
        derivative = (error - previous_error) / dt
    else:
        derivative = 0.0

    correction = (
        KP * error
        + KI * integral
        + KD * derivative
    )

    previous_error = error

    return clamp(correction, -4.0, 4.0)


def follow_corridor(client, fl, fr, dt, speed):
    '''
    Purpose:
    ---
    Drive forward while keeping the robot approximately centred between
    the available side walls.

    Input Arguments:
    ---
    `client` : [ mqtt.Client ]
        MQTT client used for wheel commands.
    `fl` : [ float ]
        Left-facing sensor distance.
    `fr` : [ float ]
        Right-facing sensor distance.
    `dt` : [ float ]
        Simulation time step in seconds.
    `speed` : [ float ]
        Desired common forward wheel speed.

    Returns:
    ---
    `left_velocity` : [ float ]
        Command sent to the left wheel.
    `right_velocity` : [ float ]
        Command sent to the right wheel.

    Example call:
    ---
    follow_corridor(client, fl, fr, dt, BASE_SPEED)
    '''

    correction = calculate_pid_correction(fl, fr, dt)

    left_velocity = clamp(speed + correction, 0.0, MAX_SPEED)
    right_velocity = clamp(speed - correction, 0.0, MAX_SPEED)

    send_velocity(client, left_velocity, right_velocity)

    return left_velocity, right_velocity


def calculate_drive_speed(front_distance):
    '''
    Purpose:
    ---
    Calculate forward speed from the remaining distance ahead.

    Input Arguments:
    ---
    `front_distance` : [ float ]
        Conservative forward obstacle distance.

    Returns:
    ---
    `speed` : [ float ]
        Forward wheel command selected for the current distance.

    Example call:
    ---
    speed = calculate_drive_speed(front_distance)
    '''

    if front_distance >= FRONT_FAST_DISTANCE:
        return BASE_SPEED

    if front_distance <= FRONT_TURN_DISTANCE:
        return TURN_START_OUTER_SPEED

    distance_range = FRONT_FAST_DISTANCE - FRONT_TURN_DISTANCE
    progress = (
        front_distance - FRONT_TURN_DISTANCE
    ) / distance_range

    progress = clamp(progress, 0.0, 1.0)

    return (
        TURN_START_OUTER_SPEED
        + progress * (APPROACH_SPEED - TURN_START_OUTER_SPEED)
    )



def choose_turn_direction(fl, fr):
    '''
    Purpose:
    ---
    Select a turn only when a genuine side opening is visible. Equal side
    readings are treated as ambiguous rather than as evidence for a turn.

    Input Arguments:
    ---
    `fl` : [ float ]
        Current left-facing sensor distance.
    `fr` : [ float ]
        Current right-facing sensor distance.

    Returns:
    ---
    `direction` : [ int ]
        -1 for left, +1 for right, or 0 when neither side is open.

    Example call:
    ---
    direction = choose_turn_direction(fl, fr)
    '''

    global junction_left_reference
    global junction_right_reference

    left_open = (
        fl >= SIDE_OPEN_DISTANCE
        or fl - junction_left_reference >= JUNCTION_OPENING_DELTA
    )
    right_open = (
        fr >= SIDE_OPEN_DISTANCE
        or fr - junction_right_reference >= JUNCTION_OPENING_DELTA
    )

    if left_open and right_open:
        left_change = fl - junction_left_reference
        right_change = fr - junction_right_reference
        return -1 if left_change >= right_change else 1

    if left_open:
        return -1

    if right_open:
        return 1

    return 0

def start_turn(direction, front_distance=0.0):
    '''
    Purpose:
    ---
    Initialise a smooth curved turn and lock its direction.

    Input Arguments:
    ---
    `direction` : [ int ]
        -1 for left or +1 for right.

    Returns:
    ---
    None.

    Example call:
    ---
    start_turn(-1)
    '''

    global state
    global turn_direction
    global turn_start_front_distance
    global turn_angle
    global turn_diagnostic_timer

    reset_pid()
    reset_turn()

    global opening_wait_timer
    global diagnostic_timer

    opening_wait_timer = 0.0
    diagnostic_timer = 0.0
    turn_direction = direction
    turn_start_front_distance = front_distance
    turn_angle = 0.0
    turn_diagnostic_timer = 0.0

    if direction < 0:
        state = STATE_TURN_LEFT
    elif direction > 0:
        state = STATE_TURN_RIGHT
    else:
        state = STATE_HALT



def turn_is_aligned():
    '''
    Purpose:
    ---
    Check whether the integrated gyro angle is close enough to 90 degrees.

    Input Arguments:
    ---
    None.

    Returns:
    ---
    `is_aligned` : [ bool ]
        True when the commanded turn has reached its angular target.

    Example call:
    ---
    if turn_is_aligned():
        ...
    '''

    return turn_angle >= TURN_TARGET_ANGLE - TURN_ANGLE_TOLERANCE

def perform_turn(client, fl, fr, sl, sr, yaw_rate, dt):
    '''
    Purpose:
    ---
    Execute a smooth, gyro-controlled 90-degree turn using differential
    wheel speeds. The wheel-speed difference decreases near the target.

    Input Arguments:
    ---
    `client` : [ mqtt.Client ]
        MQTT client used for wheel commands.
    `fl`, `fr`, `sl`, `sr` : [ float ]
        Side and forward ToF readings.
    `yaw_rate` : [ float ]
        Z-axis gyro angular velocity in radians per second.
    `dt` : [ float ]
        Simulation time step in seconds.

    Returns:
    ---
    None.

    Example call:
    ---
    perform_turn(client, fl, fr, sl, sr, yaw_rate, dt)
    '''

    global state
    global turn_timer
    global turn_angle
    global straighten_timer
    global clearance_distance

    global turn_diagnostic_timer

    turn_timer += dt
    turn_angle += abs(yaw_rate) * dt
    turn_diagnostic_timer += dt

    if turn_diagnostic_timer >= 0.25:
        print(
            f"[turn] t={turn_timer:.2f}s "
            f"angle={math.degrees(turn_angle):.1f} deg "
            f"yaw_rate={yaw_rate:.3f} rad/s"
        )
        turn_diagnostic_timer = 0.0
    angle_remaining = max(0.0, TURN_TARGET_ANGLE - turn_angle)

    # Reduce the differential wheel command as the target approaches.
    differential = clamp(
        TURN_ANGLE_KP * angle_remaining,
        TURN_MIN_SPEED,
        TURN_OUTER_SPEED - TURN_BASE_SPEED
    )

    if angle_remaining <= TURN_ANGLE_TOLERANCE:
        send_velocity(client, 0.0, 0.0)
        print(f"[turn] 90-degree turn reached ({math.degrees(turn_angle):.1f} deg).")
        reset_turn()
        straighten_timer = 0.0
        clearance_distance = 0.0
        state = STATE_STRAIGHTEN
        return

    if turn_direction > 0:
        left_velocity = TURN_BASE_SPEED + differential
        right_velocity = TURN_BASE_SPEED - differential
    else:
        left_velocity = TURN_BASE_SPEED - differential
        right_velocity = TURN_BASE_SPEED + differential

    send_velocity(client, left_velocity, right_velocity)

    if turn_timer >= TURN_MAX_TIME:
        print(
            f"[turn] WARNING: 90-degree target not reached; "
            f"gyro estimate={math.degrees(turn_angle):.1f} deg. "
            "Stopping for safety."
        )
        stop_robot(client)
        state = STATE_HALT

def perform_entry(client, dt):
    '''
    Purpose:
    ---
    Move straight through the starting region before enabling corridor
    centring and junction decisions.

    Input Arguments:
    ---
    `client` : [ mqtt.Client ]
        MQTT client used for wheel commands.
    `dt` : [ float ]
        Simulation time step in seconds.

    Returns:
    ---
    None.

    Example call:
    ---
    perform_entry(client, dt)
    '''

    global state
    global entry_timer

    entry_timer += dt

    send_velocity(client, BASE_SPEED, BASE_SPEED)

    if entry_timer >= ENTRY_TIME:
        entry_timer = 0.0
        reset_pid()
        reset_segment()
        state = STATE_CALCULATE


def perform_calculate(client, front_distance):
    '''
    Purpose:
    ---
    Calculate the usable forward distance for the next corridor and select
    the appropriate movement phase.

    Input Arguments:
    ---
    `client` : [ mqtt.Client ]
        MQTT client used for a possible safety stop.
    `front_distance` : [ float ]
        Conservative forward distance.

    Returns:
    ---
    None.

    Example call:
    ---
    perform_calculate(client, front_distance)
    '''

    global state
    global available_distance

    available_distance = calculate_available_distance(front_distance)

    if front_distance <= FRONT_TURN_DISTANCE:
        state = STATE_APPROACH
    else:
        state = STATE_FAST_MOVE


def perform_fast_move(client, fl, fr, front_distance, dt):
    '''
    Purpose:
    ---
    Move quickly through a clear corridor while updating the available
    forward distance and an estimated distance travelled.

    Input Arguments:
    ---
    `client` : [ mqtt.Client ]
        MQTT client used for wheel commands.
    `fl` : [ float ]
        Left-facing sensor distance.
    `fr` : [ float ]
        Right-facing sensor distance.
    `front_distance` : [ float ]
        Conservative forward distance.
    `dt` : [ float ]
        Simulation time step in seconds.

    Returns:
    ---
    None.

    Example call:
    ---
    perform_fast_move(client, fl, fr, front_distance, dt)
    '''

    global state
    global segment_distance
    global available_distance
    global previous_front_distance

    available_distance = calculate_available_distance(front_distance)

    left_velocity, right_velocity = follow_corridor(
        client,
        fl,
        fr,
        dt,
        BASE_SPEED
    )

    # Relative command-distance estimate; ToF remains authoritative.
    segment_distance += (
        (left_velocity + right_velocity) * 0.5 * dt
    )

    previous_front_distance = front_distance

    if front_distance <= FRONT_APPROACH_DISTANCE:
        state = STATE_APPROACH



def perform_approach(client, fl, fr, sl, sr, dt):
    '''
    Purpose:
    ---
    Slow PacBot as it approaches a junction. Once the front is close enough,
    enter a separate waiting phase so the side sensors have time to reveal
    the actual opening before a turn is selected.

    Input Arguments:
    ---
    `client` : [ mqtt.Client ]
        MQTT client used for wheel commands.
    `fl` : [ float ]
        Left-facing sensor distance.
    `fr` : [ float ]
        Right-facing sensor distance.
    `sl` : [ float ]
        Forward-left sensor distance.
    `sr` : [ float ]
        Forward-right sensor distance.
    `dt` : [ float ]
        Simulation time step in seconds.

    Returns:
    ---
    None.

    Example call:
    ---
    perform_approach(client, fl, fr, sl, sr, dt)
    '''

    global state
    global segment_distance
    global available_distance

    # Reconstruct the conservative forward distance from the two
    # forward-facing sensors. The value is local to this callback and
    # must be calculated before it is used below.
    front_distance = get_front_distance(sl, sr)
    available_distance = calculate_available_distance(front_distance)

    speed = calculate_drive_speed(front_distance)

    left_velocity, right_velocity = follow_corridor(
        client,
        fl,
        fr,
        dt,
        speed
    )

    segment_distance += (
        (left_velocity + right_velocity) * 0.5 * dt
    )

    if front_distance <= FRONT_TURN_DISTANCE:
        global junction_left_reference
        global junction_right_reference
        global junction_open_timer

        reset_pid()
        junction_left_reference = fl
        junction_right_reference = fr
        junction_open_timer = 0.0
        state = STATE_WAIT_FOR_OPENING

def perform_wait_for_opening(client, fl, fr, front_distance, dt):
    '''
    Purpose:
    ---
    Advance slowly through a junction until a persistent side opening is
    detected. Equal FL and FR values are treated as ambiguous, not as a
    reason to turn left or right.

    Input Arguments:
    ---
    `client` : [ mqtt.Client ]
        MQTT client used for wheel commands.
    `fl` : [ float ]
        Left-facing sensor distance.
    `fr` : [ float ]
        Right-facing sensor distance.
    `front_distance` : [ float ]
        Conservative forward distance.
    `dt` : [ float ]
        Simulation time step in seconds.

    Returns:
    ---
    None.

    Example call:
    ---
    perform_wait_for_opening(client, fl, fr, front_distance, dt)
    '''

    global opening_wait_timer
    global diagnostic_timer
    global junction_open_timer

    opening_wait_timer += dt
    diagnostic_timer += dt

    direction = choose_turn_direction(fl, fr)

    if direction != 0:
        junction_open_timer += dt
    else:
        junction_open_timer = 0.0

    if diagnostic_timer >= 0.10:
        print(
            f"[junction] FL={fl:.3f} FR={fr:.3f} "
            f"front={front_distance:.3f} "
            f"wait={opening_wait_timer:.2f}s"
        )
        diagnostic_timer = 0.0

    if direction != 0 and junction_open_timer >= JUNCTION_CONFIRM_TIME:
        print(
            f"[junction] {'LEFT' if direction < 0 else 'RIGHT'} opening confirmed. "
            f"FL={fl:.3f} FR={fr:.3f}"
        )

        opening_wait_timer = 0.0
        diagnostic_timer = 0.0
        junction_open_timer = 0.0
        start_turn(direction, front_distance)
        return

    # No arbitrary left/right fallback is allowed. The robot creeps forward
    # until the side sensor actually shows a persistent opening.
    follow_corridor(
        client,
        fl,
        fr,
        dt,
        TURN_START_OUTER_SPEED
    )

def perform_straighten(client, fl, fr, front_distance, dt):
    '''
    Purpose:
    ---
    Briefly settle the robot after the gyro-controlled turn.

    Input Arguments:
    ---
    `client` : [ mqtt.Client ]
        MQTT client used for wheel commands.
    `fl`, `fr` : [ float ]
        Side-facing sensor distances.
    `front_distance` : [ float ]
        Conservative forward distance.
    `dt` : [ float ]
        Simulation time step in seconds.

    Returns:
    ---
    None.

    Example call:
    ---
    perform_straighten(client, fl, fr, front_distance, dt)
    '''

    global state
    global straighten_timer
    global clearance_distance

    straighten_timer += dt
    send_velocity(client, STRAIGHTEN_SPEED, STRAIGHTEN_SPEED)

    if straighten_timer >= STRAIGHTEN_TIME:
        straighten_timer = 0.0
        clearance_distance = 0.0
        reset_pid()
        state = STATE_CLEARANCE


def perform_clearance(client, fl, fr, sl, sr, dt):
    '''
    Purpose:
    ---
    Travel a short calibrated distance after a turn before resuming
    normal corridor and junction detection.

    Input Arguments:
    ---
    `client` : [ mqtt.Client ]
        MQTT client used for wheel commands.
    `fl`, `fr`, `sl`, `sr` : [ float ]
        Current ToF sensor readings.
    `dt` : [ float ]
        Simulation time step in seconds.

    Returns:
    ---
    None.

    Example call:
    ---
    perform_clearance(client, fl, fr, sl, sr, dt)
    '''

    global state
    global clearance_distance

    front_distance = get_front_distance(sl, sr)

    # Safety: do not continue the nominal clearance movement toward a wall.
    if front_distance <= FRONT_TURN_DISTANCE:
        print(
            f"[clearance] Front obstacle at {front_distance:.3f} m; "
            "returning to approach phase."
        )
        state = STATE_APPROACH
        return

    left_velocity, right_velocity = follow_corridor(
        client, fl, fr, dt, CLEARANCE_SPEED
    )

    average_wheel_speed = (left_velocity + right_velocity) * 0.5
    clearance_distance += abs(average_wheel_speed) * WHEEL_RADIUS_M * dt

    if clearance_distance >= CLEARANCE_DISTANCE:
        print(
            f"[clearance] Travelled approximately {clearance_distance:.3f} m; "
            "resuming corridor following."
        )
        clearance_distance = 0.0
        reset_pid()
        reset_segment()
        state = STATE_CALCULATE

def on_message(client, userdata, msg):
    '''
    Purpose:
    ---
    Receive PacBot sensor data and execute the active controller phase.

    Input Arguments:
    ---
    `client` : [ mqtt.Client ]
        MQTT client receiving the message.
    `userdata` : [ object ]
        MQTT user data passed by the client library.
    `msg` : [ mqtt.MQTTMessage ]
        Incoming sensor message from pacbot/sensors.

    Returns:
    ---
    None.

    Example call:
    ---
    client.on_message = on_message
    '''

    global available_distance

    try:
        data = json.loads(msg.payload.decode())

        fl, fr, sl, sr, yaw_rate, dt = get_sensor_values(data)

        front_distance = get_front_distance(sl, sr)
        available_distance = calculate_available_distance(front_distance)

        # Gyro is retained for diagnostics; no fixed-angle turn is used.
        _ = yaw_rate

        if state == STATE_ENTRY:
            perform_entry(client, dt)
            return

        if state == STATE_CALCULATE:
            perform_calculate(client, front_distance)
            return

        if state == STATE_FAST_MOVE:
            perform_fast_move(
                client,
                fl,
                fr,
                front_distance,
                dt
            )
            return

        if state == STATE_APPROACH:
            perform_approach(
                client,
                fl,
                fr,
                sl,
                sr,
                dt
            )
            return

        if state == STATE_WAIT_FOR_OPENING:
            perform_wait_for_opening(
                client,
                fl,
                fr,
                front_distance,
                dt
            )
            return

        if state in (STATE_TURN_LEFT, STATE_TURN_RIGHT):
            perform_turn(
                client,
                fl,
                fr,
                sl,
                sr,
                yaw_rate,
                dt
            )
            return

        if state == STATE_STRAIGHTEN:
            perform_straighten(
                client,
                fl,
                fr,
                front_distance,
                dt
            )
            return

        if state == STATE_CLEARANCE:
            perform_clearance(
                client,
                fl,
                fr,
                sl,
                sr,
                dt
            )
            return

        if state == STATE_HALT:
            stop_robot(client)
            return

        stop_robot(client)

    except Exception as error:
        print(f"Sensor callback error: {error}")
        stop_robot(client)


def on_connect(client, userdata, flags, reason_code, properties=None):
    '''
    Purpose:
    ---
    Subscribe to the PacBot sensor topic after connecting to the MQTT broker.

    Input Arguments:
    ---
    `client` : [ mqtt.Client ]
        MQTT client connected to the broker.
    `userdata` : [ object ]
        MQTT user data configured for the client.
    `flags` : [ dict ]
        MQTT connection flags.
    `reason_code` : [ object ]
        MQTT connection result.
    `properties` : [ object ]
        MQTT connection properties when supported by Paho.

    Returns:
    ---
    None.

    Example call:
    ---
    client.on_connect = on_connect
    '''

    if reason_code == 0:
        print("Connected to MQTT broker.")
        client.subscribe(TOPIC_SENSORS)
        print(f"Subscribed to {TOPIC_SENSORS}")
    else:
        print(f"MQTT connection failed: {reason_code}")


def main():
    '''
    Purpose:
    ---
    Create the MQTT client, start the PacBot controller and keep the sensor
    callback active until the program is interrupted.

    Input Arguments:
    ---
    None.

    Returns:
    ---
    None.

    Example call:
    ---
    main()
    '''

    client = _mqtt_client()

    client.on_connect = on_connect
    client.on_message = on_message

    print(
        f"Connecting to MQTT broker "
        f"{MQTT_HOST}:{MQTT_PORT}..."
    )

    client.connect(
        MQTT_HOST,
        MQTT_PORT,
        60
    )

    print("PacBot controller started.")
    print(f"Base speed      : {BASE_SPEED}")
    print(f"Approach speed  : {APPROACH_SPEED}")
    print(f"Turn outer      : {TURN_OUTER_SPEED}")
    print(f"Turn inner      : {TURN_INNER_SPEED}")
    print(f"Front fast      : {FRONT_FAST_DISTANCE} m")
    print(f"Front approach  : {FRONT_APPROACH_DISTANCE} m")
    print(f"Front turn      : {FRONT_TURN_DISTANCE} m")
    print(f"Side opening    : {SIDE_OPEN_DISTANCE} m")
    print(f"Turn target     : {math.degrees(TURN_TARGET_ANGLE):.0f} deg")
    print(f"Clearance target: {CLEARANCE_DISTANCE} m")

    try:
        client.loop_forever()

    except KeyboardInterrupt:
        print("\nStopping PacBot...")
        stop_robot(client)

    finally:
        client.disconnect()


if __name__ == "__main__":
    main()
