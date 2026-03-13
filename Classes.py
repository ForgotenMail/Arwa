# Shared utility classes/functions used across the project.
# These are code-level helpers and not physical declarations.

# Shared math constant for geometry helpers.
PI = 3.1415926535



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


# Simple mutable 2D point object.
class Point:
    # Build a point with x/y coordinates.
    def __init__(self, x, y):
        self.x = x
        self.y = y

    # Getter for x.
    def get_x(self):
        return self.x

    # Getter for y.
    def get_y(self):
        return self.y

    # Setter for x.
    def set_x(self, x):
        self.x = x

    # Setter for y.
    def set_y(self, y):
        self.y = y

    # Increment point x coordinate.
    def increase_x(self, amount):
        self.x += amount

    # Increment point y coordinate.
    def increase_y(self, amount):
        self.y += amount


# Clamp value into [low, high].
def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


# Return absolute value without external libraries.
def abs_value(value):
    if value < 0:
        return -value
    return value


# Compute square root using Newton's method.
def sqrt_value(value):
    # Guard against negative/zero inputs.
    if value <= 0:
        return 0

    # Seed guess with input value.
    guess = value
    i = 0
    # Run a fixed number of refinement steps.
    while i < 12:
        guess = 0.5 * (guess + (value / guess))
        i += 1
    return guess


# Compute Euclidean distance between two points.
def distance_between(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    return sqrt_value((dx * dx) + (dy * dy))


# Normalize angle into [-180, 180].
def normalize_angle_deg(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


# Approximate atan(z) for |z| roughly <= 1 using a short series.
def _atan_small(z):
    z2 = z * z
    return z - ((z2 * z) / 3) + ((z2 * z2 * z) / 5)


# Approximate atan2(y, x) and return degrees.
def atan2_deg(y, x):
    # Handle vertical-axis and origin edge cases.
    if x == 0:
        if y > 0:
            return 90
        if y < 0:
            return -90
        return 0

    # Compute slope for atan approximation.
    z = y / x

    # Use direct approximation for moderate slopes.
    if abs_value(z) <= 1:
        angle = _atan_small(z) * (180 / PI)
        if x < 0:
            if y >= 0:
                return angle + 180
            return angle - 180
        return angle

    # Use atan(z) = 90 - atan(1/z) for steep slopes.
    angle = 90 - (_atan_small(1 / z) * (180 / PI))
    if y < 0:
        angle -= 180
    return angle


# Get (x, y) from either Point-like object or tuple/list.
def get_xy(point):
    if hasattr(point, "x") and hasattr(point, "y"):
        return point.x, point.y
    return point[0], point[1]


# Project a point onto a path segment.
def segment_projection(start, end, point):
    # Unpack start, end, and query points.
    sx, sy = start
    ex, ey = end
    px, py = point

    # Build the segment vector.
    dx = ex - sx
    dy = ey - sy
    seg_len2 = (dx * dx) + (dy * dy)

    # If segment is degenerate, return the start point.
    if seg_len2 == 0:
        return start

    # Compute normalized distance along segment.
    t = (((px - sx) * dx) + ((py - sy) * dy)) / seg_len2
    # Keep the result inside segment endpoints.
    t = clamp(t, 0, 1)

    # Return projected coordinate.
    return (sx + (t * dx), sy + (t * dy))


# Find the closest point on a polyline path to a position.
def closest_point(path, position):
    # Default best result from path start.
    best_idx = 0
    best_point = path[0]
    best_dist = 999999999

    # Evaluate each segment projection.
    i = 0
    while i < len(path) - 1:
        projected = segment_projection(path[i], path[i + 1], position)
        d = distance_between(projected[0], projected[1], position[0], position[1])
        if d < best_dist:
            best_dist = d
            best_idx = i
            best_point = projected
        i += 1

    # Return segment index and projected point.
    return best_idx, best_point


# Compute factorial using iterative multiplication.
def factorial(value):
    if value <= 1:
        return 1
    total = 1
    i = 2
    while i <= value:
        total *= i
        i += 1
    return total


# Convert degrees to radians.
def deg_to_rad(degrees):
    return degrees * (PI / 180)


# Convert radians to degrees.
def rad_to_deg(radians):
    return radians * (180 / PI)


# Approximate sine using short Taylor series around zero.
def sin_deg(degrees):
    x = deg_to_rad(degrees)
    # Wrap input to improve approximation stability.
    while x > PI:
        x -= 2 * PI
    while x < -PI:
        x += 2 * PI

    x2 = x * x
    return x - ((x2 * x) / 6) + ((x2 * x2 * x) / 120)


# Approximate cosine using short Taylor series around zero.
def cos_deg(degrees):
    x = deg_to_rad(degrees)
    # Wrap input to improve approximation stability.
    while x > PI:
        x -= 2 * PI
    while x < -PI:
        x += 2 * PI

    x2 = x * x
    return 1 - (x2 / 2) + ((x2 * x2) / 24)
