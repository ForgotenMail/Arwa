# Motion algorithm implementations.
# This file should contain algorithm application logic only.

from drivetrains import Gyro, GearRatio, WheelDiamater, drive
from Classes import clamp, distance_between, normalize_angle_deg, atan2_deg, get_xy, abs_value

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


# Drive a target linear distance while correcting heading with PID.
def LinearPID(distance_inches, speed_volts, target_heading_deg, max_steps=500):
    # Save starting encoder position.
    start_ticks = drive.MotorPosition()
    # Store previous heading error for derivative term.
    previous_error = 0
    # Store integrated heading error for integral term.
    integral = 0
    # Loop counter used as a safety stop.
    steps = 0

    # Run until distance goal or loop safety limit is reached.
    while steps < max_steps:
        # Measure current heading and compute heading error.
        heading = _get_heading_degrees()
        error = normalize_angle_deg(target_heading_deg - heading)

        # Update PID helper terms.
        integral += error
        derivative = error - previous_error
        previous_error = error

        # Combine PID terms into steering correction.
        correction = (LinearKp * error) + (LinearKi * integral) + (LinearKd * derivative)
        # Apply forward command plus turn correction.
        drive.drive_tank(clamp(speed_volts, -12, 12), clamp(correction, -12, 12))

        # Check distance completion from encoder feedback.
        driven = abs_value(_motor_distance_inches(start_ticks, drive.MotorPosition()))
        if driven >= abs_value(distance_inches):
            break

        # Increment loop safety counter.
        steps += 1

    # Stop drivetrain at the end of the routine.
    drive.drive_tank(0, 0)


# Turn to a target heading using angular PID.
def AngularPID(speed_volts, target_heading_deg, buffer_deg=1.5, max_steps=400):
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
        turn_cmd = clamp(pid * speed_volts, -12, 12)
        # Command an in-place turn.
        drive.drive_tank(0, turn_cmd)

        # Stop once within angular tolerance.
        if abs_value(error) <= buffer_deg:
            break

        # Increment loop safety counter.
        steps += 1

    # Stop drivetrain at the end of the routine.
    drive.drive_tank(0, 0)


# Project a point onto the line segment [start, end].
def _segment_projection(start, end, point):
    # Unpack endpoint and query point coordinates.
    sx, sy = start
    ex, ey = end
    px, py = point

    # Compute segment direction vector.
    dx = ex - sx
    dy = ey - sy
    seg_len2 = (dx * dx) + (dy * dy)

    # Handle zero-length segment safely.
    if seg_len2 == 0:
        return start

    # Compute normalized projection value along the segment.
    t = (((px - sx) * dx) + ((py - sy) * dy)) / seg_len2
    # Clamp projection to remain on the segment.
    t = clamp(t, 0, 1)
    # Return projected coordinate.
    return (sx + (t * dx), sy + (t * dy))


# Find the closest point to robot position on a polyline path.
def _closest_point(path, position):
    # Default best values from path start.
    best_idx = 0
    best_point = path[0]
    best_dist = 999999999

    # Check each segment projection and keep the nearest result.
    i = 0
    while i < len(path) - 1:
        projected = _segment_projection(path[i], path[i + 1], position)
        d = distance_between(projected[0], projected[1], position[0], position[1])
        if d < best_dist:
            best_dist = d
            best_idx = i
            best_point = projected
        i += 1

    # Return segment index and closest projected coordinate.
    return best_idx, best_point


# Follow a path using pure pursuit lookahead steering.
def PurePursuit(path, tracker, speed_volts=6, lookahead_inches=8, heading_kp=0.08, stop_tolerance_inches=1, max_steps=1200):
    # Normalize input points to (x, y) tuples.
    clean_path = []
    for p in path:
        x, y = get_xy(p)
        clean_path.append((x, y))

    # Require at least one segment to follow.
    if len(clean_path) < 2:
        return

    # Clamp commanded speed once per routine.
    speed_cmd = clamp(speed_volts, -12, 12)
    # Loop counter used as a safety stop.
    steps = 0

    # Continue path following until the end tolerance is met.
    while steps < max_steps:
        # Read robot position from the provided tracker.
        robot_pos = tracker.get_pos()
        rx, ry = get_xy(robot_pos)

        # Find the nearest point on the path.
        closest_segment, closest = _closest_point(clean_path, (rx, ry))

        # Walk forward from nearest point by lookahead distance.
        remaining = lookahead_inches
        lookahead = closest
        i = closest_segment
        current = closest

        # March across segments until lookahead point is found.
        while i < len(clean_path) - 1:
            nxt = clean_path[i + 1]
            seg_len = distance_between(current[0], current[1], nxt[0], nxt[1])

            # Interpolate inside this segment when lookahead lands here.
            if seg_len >= remaining and seg_len > 0:
                ratio = remaining / seg_len
                lookahead = (
                    current[0] + ((nxt[0] - current[0]) * ratio),
                    current[1] + ((nxt[1] - current[1]) * ratio),
                )
                break

            # Otherwise consume this full segment and continue.
            remaining -= seg_len
            current = nxt
            lookahead = nxt
            i += 1

        # Convert lookahead vector into heading target.
        target_heading = atan2_deg(lookahead[1] - ry, lookahead[0] - rx)
        # Compute wrapped heading error.
        heading_error = normalize_angle_deg(target_heading - _get_heading_degrees())
        # Convert heading error into a bounded turn command.
        turn = clamp(heading_error * heading_kp, -12, 12)
        # Drive forward with steering correction.
        drive.drive_tank(speed_cmd, turn)

        # Stop when robot reaches the path endpoint tolerance.
        end_x, end_y = clean_path[len(clean_path) - 1]
        if distance_between(rx, ry, end_x, end_y) <= stop_tolerance_inches:
            break

        # Increment loop safety counter.
        steps += 1

    # Stop drivetrain at the end of the routine.
    drive.drive_tank(0, 0)
