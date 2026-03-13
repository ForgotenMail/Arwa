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

    # Compute straight-line distance to lookahead.
    lookahead_dist = distance_between(robot_x, robot_y, lookahead_x, lookahead_y)
    if lookahead_dist == 0:
        return 0

    # Transform into robot frame using inverse heading rotation.
    x_robot = (dx * cos_deg(robot_heading_deg)) + (dy * sin_deg(robot_heading_deg))
    y_robot = (-dx * sin_deg(robot_heading_deg)) + (dy * cos_deg(robot_heading_deg))

    # Curvature for circle from robot origin to target point.
    # k = 2*y / (x^2 + y^2)
    denom = (x_robot * x_robot) + (y_robot * y_robot)
    if denom == 0:
        return 0
    return (2 * y_robot) / denom


# Drive a target linear distance while correcting heading with PID.
def LinearPID(distance_inches, speed, target_heading_deg, buffer_inches=0.5, max_steps=500):
    # Save starting encoder position.
    start_ticks = drive.MotorPosition()
    # Store previous heading error for derivative term.
    previous_error = 0
    # Store integrated heading error for integral term.
    integral = 0
    # Loop counter used as a safety stop.
    steps = 0

from drivetrains import drive, Gyro, GearRatio, WheelDiamater
from Classes import point

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

        # Reduce forward power as turning demand increases.
        # This makes correction affect how aggressively we drive forward.
        adjusted_speed = speed - abs_value(correction)
        adjusted_speed = clamp(adjusted_speed, -12, 12)

        # Drive with exactly two tank values: forward speed and turn correction.
        drive.drive_tank(adjusted_speed, correction)

        # Check distance completion from encoder feedback.
        driven = abs_value(_motor_distance_inches(start_ticks, drive.MotorPosition()))
        if driven >= (abs_value(distance_inches) - buffer_inches):
            break

        # Increment loop safety counter.
        steps += 1

    # Stop drivetrain at the end of the routine.
    drive.drive_tank(0, 0)


# Turn to a target heading using angular PID.
def AngularPID(speed, target_heading_deg, buffer_deg=1.5, max_steps=400):
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
        correction = clamp(pid * speed, -12, 12)

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


# Drive to a field point using odometry feedback and PID-style correction.
def OdomDriveToPoint(
    target_x,
    target_y,
    tracker,
    speed=6,
    heading_buffer_deg=1.5,
    distance_buffer_inches=0.5,
    max_steps=900,
):
    # Clamp requested top speed once.
    base_speed = clamp(speed, -12, 12)

    # PID memory terms for heading correction.
    previous_heading_error = 0
    heading_integral = 0

    # Safety loop counter to avoid endless loops.
    steps = 0

    # Continuously re-target using odometry until within distance tolerance.
    while steps < max_steps:
        # Read current odometry-estimated robot position.
        robot_pos = tracker.get_pos()
        robot_x, robot_y = get_xy(robot_pos)

        # Compute vector from robot to requested target point.
        delta_x = target_x - robot_x
        delta_y = target_y - robot_y

        # Compute remaining linear distance to the target point.
        remaining_distance = distance_between(robot_x, robot_y, target_x, target_y)

        # Stop once we are inside the requested distance buffer.
        if remaining_distance <= abs_value(distance_buffer_inches):
            break

        # Compute heading we should face to drive directly at the target.
        target_heading_deg = atan2_deg(delta_y, delta_x)

        # Compute wrapped heading error to avoid long-turn pathing.
        heading_error = normalize_angle_deg(target_heading_deg - _get_heading_degrees())

        # Update heading PID terms.
        heading_integral += heading_error
        heading_derivative = heading_error - previous_heading_error
        previous_heading_error = heading_error

        # Build turn correction from heading PID gains.
        correction = (AngularKp * heading_error) + (AngularKi * heading_integral) + (AngularKd * heading_derivative)
        correction = clamp(correction, -12, 12)

        # Use remaining distance to scale forward command, capped by requested speed.
        # This slows down naturally as we approach the point.
        forward_limit = clamp(remaining_distance * 0.6, 0, abs_value(base_speed))

        # If heading is far off, prioritize turning before pushing hard forward.
        if abs_value(heading_error) > 30:
            forward_limit = clamp(forward_limit * 0.35, 0, abs_value(base_speed))

        # Reduce forward when correction is high, while preserving drive direction.
        forward_mag = clamp(forward_limit - abs_value(correction), 0, abs_value(base_speed))
        forward = sign(base_speed) * forward_mag

        # Drive with exactly two values: forward and turn correction.
        drive.drive_tank(forward, correction)

        # Count this control update iteration.
        steps += 1

    # Stop the drivetrain once the routine is complete.
    drive.drive_tank(0, 0)
