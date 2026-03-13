# Shared utility classes/functions used across the project.
# These are code-level helpers and not physical declarations.

# Shared math constant for geometry helpers.
PI = 3.1415926535


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
