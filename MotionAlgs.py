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
    speed = clamp(speed, -12, 12)
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
        # Use speed for forward command and curvature-scaled correction for turn.
        correction = clamp(curvature * speed * 12, -12, 12)
        forward = speed - abs_value(correction)
        forward = clamp(forward, -12, 12)

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

        PID = P + Ipid + D

        drive.drive_tank(0, (Speed*PID)/12)

