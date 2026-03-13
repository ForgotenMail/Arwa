# Motion algorithm implementations.
# This file contains algorithm application logic only.

from drivetrains import Gyro, GearRatio, WheelDiamater, drive
from Classes import (
    clamp,
    distance_between,
    normalize_angle_deg,
    atan2_deg,
    get_xy,
    abs_value,
    sign,
    sin_deg,
    cos_deg,
    closest_point,
)

# Linear heading-hold PID gains.
LinearKp = 0.55
LinearKi = 0.0
LinearKd = 0.05

# Angular turn PID gains.
AngularKp = 0.09
AngularKi = 0.0
AngularKd = 0.004


# Convert drivetrain encoder ticks into linear inches traveled.
def _motor_distance_inches(start_ticks, current_ticks):
    # Convert raw tick delta to motor revolutions.
    motor_revolutions = (current_ticks - start_ticks) / 4096.0
    # Convert motor revolutions to wheel revolutions using configured gearing.
    wheel_revolutions = motor_revolutions * GearRatio
    # Convert wheel revolutions into inches of travel.
    return wheel_revolutions * (WheelDiamater * 3.1415926535)


# Read current heading from the configured gyro.
def _get_heading_degrees():
    # Some API variants accept no argument.
    try:
        return Gyro.get_heading()
    # Some API variants require a unit argument.
    except TypeError:
        return Gyro.get_heading("degrees")


# Compute the lookahead point by walking forward from the closest point.
def _calculate_lookahead_point(path, robot_x, robot_y, lookahead_inches):
    # Start from the closest projected point on the path.
    closest_segment, closest = closest_point(path, (robot_x, robot_y))

    # Move forward across segments by lookahead distance.
    remaining = lookahead_inches
    lookahead = closest
    i = closest_segment
    current = closest

    # Walk segment-by-segment until lookahead is located.
    while i < len(path) - 1:
        nxt = path[i + 1]
        seg_len = distance_between(current[0], current[1], nxt[0], nxt[1])

        # If lookahead falls on this segment, interpolate and stop.
        if seg_len >= remaining and seg_len > 0:
            ratio = remaining / seg_len
            lookahead = (
                current[0] + ((nxt[0] - current[0]) * ratio),
                current[1] + ((nxt[1] - current[1]) * ratio),
            )
            break

        # Otherwise consume the full segment and continue.
        remaining -= seg_len
        current = nxt
        lookahead = nxt
        i += 1

    # Return the final lookahead coordinate.
    return lookahead


# Compute pursuit arc curvature from robot pose to lookahead point.
def _calculate_arc_curvature(robot_x, robot_y, robot_heading_deg, lookahead_x, lookahead_y):
    # Build vector from robot to lookahead in field frame.
    dx = lookahead_x - robot_x
    dy = lookahead_y - robot_y

    # Transform into robot frame using inverse heading rotation.
    x_robot = (dx * cos_deg(robot_heading_deg)) + (dy * sin_deg(robot_heading_deg))
    y_robot = (-dx * sin_deg(robot_heading_deg)) + (dy * cos_deg(robot_heading_deg))

    # Curvature for circle from robot origin to target point:
    # k = 2*y / (x^2 + y^2)
    denom = (x_robot * x_robot) + (y_robot * y_robot)
    if denom == 0:
        return 0
    return (2 * y_robot) / denom


# Drive a target linear distance while correcting heading with PID.
def LinearPID(distance_inches, speed, target_heading_deg, buffer_inches=0.5, max_steps=500):
    # Save starting encoder position.
    start_ticks = drive.MotorPosition()
    # Clamp requested speed to valid range once.
    base_speed = clamp(speed, -12, 12)
    # Store previous heading error for derivative term.
    previous_error = 0
    # Store integrated heading error for integral term.
    integral = 0
    # Loop counter used as a safety stop.
    steps = 0

    # Build a safe distance threshold that cannot go below zero.
    distance_target = abs_value(distance_inches)
    distance_buffer = abs_value(buffer_inches)
    distance_threshold = clamp(distance_target - distance_buffer, 0, distance_target)

    # Run until distance goal (+ buffer) or safety limit is reached.
    while steps < max_steps:
        # Measure current heading and compute wrapped heading error.
        heading = _get_heading_degrees()
        error = normalize_angle_deg(target_heading_deg - heading)

        # Update PID helper terms.
        integral += error
        derivative = error - previous_error
        previous_error = error

        # Combine PID terms into steering correction.
        correction = (LinearKp * error) + (LinearKi * integral) + (LinearKd * derivative)
        correction = clamp(correction, -12, 12)

        # Reduce forward magnitude as turning demand increases.
        # Keep the original drive direction (forward/reverse) from speed sign.
        forward_mag = clamp(abs_value(base_speed) - abs_value(correction), 0, 12)
        forward = sign(base_speed) * forward_mag

        # Drive with exactly two tank values: forward speed and turn correction.
        drive.drive_tank(forward, correction)

        # Check distance completion from encoder feedback.
        driven = abs_value(_motor_distance_inches(start_ticks, drive.MotorPosition()))
        if driven >= distance_threshold:
            break

        # Increment loop safety counter.
        steps += 1

    # Stop drivetrain at the end of the routine.
    drive.drive_tank(0, 0)


# Turn to a target heading using angular PID.
def AngularPID(speed, target_heading_deg, buffer_deg=1.5, max_steps=400):
    # Clamp requested speed magnitude for turn scaling.
    turn_scale = abs_value(clamp(speed, -12, 12))
    # Store previous heading error for derivative term.
    previous_error = 0
    # Store integrated heading error for integral term.
    integral = 0
    # Loop counter used as a safety stop.
    steps = 0

    # Run until angular buffer is met or safety limit is reached.
    while steps < max_steps:
        # Measure current heading and compute shortest-turn error.
        heading = _get_heading_degrees()
        error = normalize_angle_deg(target_heading_deg - heading)

        # Update PID helper terms.
        integral += error
        derivative = error - previous_error
        previous_error = error

        # Build turn output and clamp to drivetrain range.
        pid = (AngularKp * error) + (AngularKi * integral) + (AngularKd * derivative)
        correction = clamp(pid * turn_scale, -12, 12)

        # Drive with exactly two tank values: forward speed and turn correction.
        drive.drive_tank(0, correction)

        # Stop once within angular tolerance.
        if abs_value(error) <= buffer_deg:
            break

        # Increment loop safety counter.
        steps += 1

    # Stop drivetrain at the end of the routine.
    drive.drive_tank(0, 0)


# Follow a path using pure pursuit with explicit readable steps.
def PurePursuit(path, tracker, speed=6, lookahead_inches=8, stop_tolerance_inches=1, max_steps=1200):
    # Convert input points into a clean tuple list.
    clean_path = []
    for p in path:
        x, y = get_xy(p)
        clean_path.append((x, y))

    # Require at least one line segment.
    if len(clean_path) < 2:
        return

    # Clamp base forward speed once.
    base_speed = clamp(speed, -12, 12)
    steps = 0

    # Run path following loop.
    while steps < max_steps:
        # Read robot pose from tracker and gyro.
        robot_pos = tracker.get_pos()
        robot_x, robot_y = get_xy(robot_pos)
        robot_heading_deg = _get_heading_degrees()

        # STEP 1: Calculate lookahead point on the path.
        lookahead_x, lookahead_y = _calculate_lookahead_point(clean_path, robot_x, robot_y, lookahead_inches)

        # STEP 2: Calculate the arc curvature from robot to lookahead point.
        curvature = _calculate_arc_curvature(robot_x, robot_y, robot_heading_deg, lookahead_x, lookahead_y)

        # STEP 3: Convert curvature to motor drive values for forward motion.
        # Scale curvature by speed to create turn correction.
        correction = clamp(curvature * abs_value(base_speed), -12, 12)
        # Reduce forward magnitude during aggressive turns, keep direction.
        forward_mag = clamp(abs_value(base_speed) - abs_value(correction), 0, 12)
        forward = sign(base_speed) * forward_mag

        # Drive with exactly two tank values: forward speed and turn correction.
        drive.drive_tank(forward, correction)

        # Stop when close enough to path endpoint.
        end_x, end_y = clean_path[len(clean_path) - 1]
        if distance_between(robot_x, robot_y, end_x, end_y) <= stop_tolerance_inches:
            break

        # Increment loop safety counter.
        steps += 1

    # Stop drivetrain at the end of the routine.
    drive.drive_tank(0, 0)


# Drive to a field point using odometry + heading/distance PID.
def OdomDriveToPoint(target_x, target_y, tracker, speed=6, heading_buffer_deg=1.5, distance_buffer_inches=0.5):
    # Read current robot position from the tracker.
    robot_pos = tracker.get_pos()
    robot_x, robot_y = get_xy(robot_pos)

    # Build target vector from robot to requested point.
    delta_x = target_x - robot_x
    delta_y = target_y - robot_y

    # Compute angle we must face to drive straight to the point.
    target_heading_deg = atan2_deg(delta_y, delta_x)

    # Compute linear distance needed to reach the point.
    distance_inches = distance_between(robot_x, robot_y, target_x, target_y)

    # First turn to the desired heading.
    AngularPID(speed, target_heading_deg, heading_buffer_deg)

    # Then drive the computed distance while holding that heading.
    LinearPID(distance_inches, speed, target_heading_deg, distance_buffer_inches)
