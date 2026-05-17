from venice import Direction, Gearset, InertialSensor, Motor, RotationSensor, RotationUnit
from Classes import Point, sin_deg, cos_deg


# Venice API notes:
# - Motor and RotationSensor constructors both take a Smart Port number plus a Direction.
# - Encoder/rotation reads use explicit RotationUnit values where the API requires units.
# - Motor voltage commands are in volts, so drivetrain outputs are clamped to +/-12V.


"""This file is where you will define your drivetrain
The variable DriveTrainType is what defines what dirve train you have, the current drivetrains that are
supported are these (Name followed for the variable in paranthesis)
Tank/Differential any motors (tank)
X - drive 4 Motors (holomonic)
Mecanum 4 motors (holomonic)
The math behind X drive and mecanum 4 motors is the same so we just call them the same thing! """

DriveTrainType = "Null"

"""Type how many motors you have in these lists, it goes from front to back! """
LeftMotors: list = [0, 0]
RightMotors: list = [0, 0]


"""Put this into a fraction, all you have to do is motors RPM * InputGear/Output gear"""
GearRatio = 450


# Put this into Iches
WheelDiamater = 4

#Put your IMU port in
Gyro = InertialSensor(5)

#Odom, This Supports odometry, in this variable you have to specify the number of Odom pods you have. This code suports, 0, 1, and 2 Odom wheels.
# If you are using 1 Odom pod it assumes that you are using a vertical odom wheel, 2 assumes 1 vertical and 1 lateral
#   The code should be self explanitory so if you have something that is not made already, it should be easy for you to make your own
#Even if you are not using Odometry in spesific, having Odom sensors still helps PID with acuracy
AmtOdom = 0
#Put your odom rotation ports in here, Vertical first, lateral 2nd
OdomPorts = []

#Put your wheel diamater for your Odom wheels in here in inches, if you are using a geared odom pod, just adjust the diamater
OdomDiameter  = 5


# The way we define drivetrains is that each type of drivetrain is a class
# At the end, we declare our variable called "drivetrain" to be one of these classes
#   That depends on what the user puts in
# Each drivetrain contaitns the logic on how to move given Forward velocity, and how much you want to turn


def clamp_voltage(voltage):
    """Keep every Venice Motor.set_voltage command inside the legal +/-12V range."""
    return max(-12, min(12, voltage))


def tank_voltages(forward, turn):
    """Convert forward/turn requests into left/right tank-drive voltages."""
    left_voltage = clamp_voltage(forward + turn)
    right_voltage = clamp_voltage(forward - turn)
    return left_voltage, right_voltage


def _first_port(port_or_ports):
    """Allow one-odom configs to pass either a single port or a one-item port list."""
    if isinstance(port_or_ports, list):
        return port_or_ports[0]
    return port_or_ports


def create_motor(port: int) -> Motor:
    """Create a Venice Motor, using a negative port number to mark it reversed."""
    port_abs: int = abs(port)
    if port < 0:
        # Venice expects the reversal as Direction.REVERSE, not as a negative port.
        return Motor(port_abs, Direction.REVERSE, Gearset.GREEN)
    else:
        return Motor(port_abs, Direction.FORWARD, Gearset.GREEN)


def create_rotation(port: int) -> RotationSensor:
    """Create a Venice RotationSensor, using a negative port number to mark it reversed."""
    port_abs: int = abs(port)
    if port < 0:
        # RotationSensor follows the same Direction enum pattern as Motor.
        return RotationSensor(port_abs, Direction.REVERSE)
    else:
        return RotationSensor(port_abs, Direction.FORWARD)

class TankDrivetrain:
    def __init__(self, left_ports: list[int], right_ports: list[int]):
        self.left_motors = [create_motor(p) for p in left_ports]
        self.right_motors = [create_motor(p) for p in right_ports]

    def drive_tank(self, velocity, turn: int):
        # These for loops just say for each motor on each side, move the motor however fast forward
        # and how fast you want to turn. This means regardless of our drivetrain type,
        #  we can call the same function!

        left_voltage, right_voltage = tank_voltages(velocity, turn)

        for m in self.left_motors:
            m.set_voltage(left_voltage)
        for m in self.right_motors:
            m.set_voltage(right_voltage)

    def MotorPosition(self):
        # Venice exposes raw encoder ticks with get_raw_position().
        Amt_Motors = 0
        Total = 0
        for m in self.left_motors + self.right_motors:
            Amt_Motors += 1
            Total += m.get_raw_position()
        return Total / Amt_Motors


class HolomonicDrive:
    def __init__(self, left_ports: list[int], right_ports: list[int]):
        self.LeftFront = create_motor(left_ports[0])
        self.LeftBack = create_motor(left_ports[1])
        self.RightFront = create_motor(right_ports[0])
        self.RightBack = create_motor(right_ports[1])

    def drive_tank(self, Forward, Rotate):
        # Reuse tank math so autonomous routines can call drive_tank on any drive type.
        FL, FR = tank_voltages(Forward, Rotate)
        BL, BR = FL, FR

        self.LeftFront.set_voltage(FL)
        self.RightFront.set_voltage(FR)
        self.LeftBack.set_voltage(BL)
        self.RightBack.set_voltage(BR)

    def drive_holomonic(self, Forward, Lateral, Rotate):
        # X-drive/mecanum mixing: combine forward, strafe, and turn, then clamp volts.
        FL = clamp_voltage(Forward + Lateral + Rotate)
        FR = clamp_voltage(Forward - Lateral - Rotate)
        BL = clamp_voltage(Forward - Lateral + Rotate)
        BR = clamp_voltage(Forward + Lateral - Rotate)

        self.LeftFront.set_voltage(FL)
        self.RightFront.set_voltage(FR)
        self.LeftBack.set_voltage(BL)
        self.RightBack.set_voltage(BR)

    def MotorPosition(self):
        # Average all motor encoders so PID distance checks use one drivetrain value.
        Total = 0
        Total += self.LeftFront.get_raw_position()
        Total += self.RightFront.get_raw_position()
        Total += self.RightBack.get_raw_position()
        Total += self.LeftBack.get_raw_position()

        return Total / 4


class HolomonicDrive6:
    def __init__(self, left_ports: list[int], right_ports: list[int]):
        self.LeftFront = create_motor(left_ports[0])
        self.LeftMiddle = create_motor(left_ports[1])
        self.LeftBack = create_motor(left_ports[2])
        self.RightFront = create_motor(right_ports[0])
        self.RightMiddle = create_motor(right_ports[1])
        self.RightBack = create_motor(right_ports[2])

    def drive_tank(self, Forward, Rotate):
        # Six-motor tank mode mirrors each side's voltage across front/middle/back.
        FL, FR = tank_voltages(Forward, Rotate)
        ML, MR = FL, FR
        BL, BR = FL, FR

        self.LeftFront.set_voltage(FL)
        self.RightFront.set_voltage(FR)
        self.LeftMiddle.set_voltage(ML)
        self.RightMiddle.set_voltage(MR)
        self.LeftBack.set_voltage(BL)
        self.RightBack.set_voltage(BR)

    def drive_holomonic(self, Forward, Lateral, Rotate):
        # Six-motor holonomic mixing keeps middle motors out of the strafe term.
        FL = clamp_voltage(Forward + Lateral + Rotate)
        FR = clamp_voltage(Forward - Lateral - Rotate)
        ML = clamp_voltage(Forward + Rotate)
        MR = clamp_voltage(Forward - Rotate)
        BL = clamp_voltage(Forward - Lateral + Rotate)
        BR = clamp_voltage(Forward + Lateral - Rotate)

        self.LeftFront.set_voltage(FL)
        self.RightFront.set_voltage(FR)
        self.LeftMiddle.set_voltage(ML)
        self.RightMiddle.set_voltage(MR)
        self.LeftBack.set_voltage(BL)
        self.RightBack.set_voltage(BR)

    def MotorPosition(self):
        # Average all six raw motor encoder positions.
        Total = 0
        Total += self.LeftFront.get_raw_position()
        Total += self.RightFront.get_raw_position()
        Total += self.RightMiddle.get_raw_position()
        Total += self.LeftMiddle.get_raw_position()
        Total += self.RightBack.get_raw_position()
        Total += self.LeftBack.get_raw_position()

        return Total / 6

class TrackingNoOdom:
    def __init__(self, Gyro, drive, GearRatio, WheelDiameter):
        self.gyro = Gyro
        self.drive = drive
        self.pos = Point(0, 0)
        self.GearRatio = GearRatio
        self.WheelDiameter = WheelDiameter

        # Store previous motor position to calculate delta
        self.prev_motor = self.drive.MotorPosition()

    # This function continuously updates the position of the robot
    # Each loop it takes the information from the motors and updates the positions
    def updatePose(self):
        while True:
            # Read current motor rotation
            curr_motor = self.drive.MotorPosition()

            # Calculate change since last update
            delta_motor = curr_motor - self.prev_motor
            Wheel_rotations = (delta_motor / 4096) * self.GearRatio
            DistanceMoved = Wheel_rotations * 3.14159 * self.WheelDiameter

            # Update global position using current heading
            # Venice requires an explicit RotationUnit for heading reads.
            heading = self.gyro.get_heading(RotationUnit.DEGREES)
            self.pos.increase_x(DistanceMoved * cos_deg(heading))
            self.pos.increase_y(DistanceMoved * sin_deg(heading))

            # Save current motor position for next loop
            self.prev_motor = curr_motor

    def get_pos(self):
        return self.pos


class Tracking1Odom:
    def __init__(self, Gyro, OdomDiameter, OdomPorts):
        self.gyro = Gyro
        self.pos = Point(0, 0)
        self.OdomDiameter = OdomDiameter
        self.OdomPod1 = create_rotation(_first_port(OdomPorts))

        # Store previous pod position to calculate delta
        # RotationSensor.get_position() also requires an explicit RotationUnit.
        self.prev_odom = self.OdomPod1.get_position(RotationUnit.DEGREES)

    # This function continuously updates the position of the robot
    # Each loop it takes the information from the odom pod and updates the positions
    def updatePose(self):
        while True:
            # Read current odom pod rotation
            curr_odom = self.OdomPod1.get_position(RotationUnit.DEGREES)

            # Calculate change since last update
            delta_rot = curr_odom - self.prev_odom
            DistanceMoved = delta_rot * self.OdomDiameter * 3.14159 / 360  # convert degrees to revolutions

            # Update global position using current heading
            heading = self.gyro.get_heading(RotationUnit.DEGREES)
            self.pos.increase_x(DistanceMoved * cos_deg(heading))
            self.pos.increase_y(DistanceMoved * sin_deg(heading))

            # Save current odom position for next loop
            self.prev_odom = curr_odom

    def get_pos(self):
        return self.pos



class Tracking2Odom:
    def __init__ (self, Gyro, OdomDiameter, OdomPorts):
        self.gyro = Gyro
        self.pos = Point(0, 0)
        self.OdomDiameter = OdomDiameter

        # Create the two odometry pods
        self.OdomPodVertical = create_rotation(OdomPorts[0])
        self.OdomPodHorizontal = create_rotation(OdomPorts[1])

        # Store previous positions for delta calculation
        # Store both odom pods in degrees so the distance math below can divide by 360.
        self.prev_vert = self.OdomPodVertical.get_position(RotationUnit.DEGREES)
        self.prev_horiz = self.OdomPodHorizontal.get_position(RotationUnit.DEGREES)

    # This function continuously updates the position of the robot
    # Each loop it takes the information from the odom pods and updates the positions
    def updatePose(self):
        while True:

            # Read current odom pod rotations
            curr_vert = self.OdomPodVertical.get_position(RotationUnit.DEGREES)
            curr_horiz = self.OdomPodHorizontal.get_position(RotationUnit.DEGREES)

            # Compute how much each pod moved since last update
            delta_vert = (curr_vert - self.prev_vert) * self.OdomDiameter * 3.14159 / 360
            delta_horiz = (curr_horiz - self.prev_horiz) * self.OdomDiameter * 3.14159 / 360

            # Rotate local movement by current heading to get field coordinates
            heading = self.gyro.get_heading(RotationUnit.DEGREES)
            self.pos.increase_x(delta_horiz * cos_deg(heading) - delta_vert * sin_deg(heading))
            self.pos.increase_y(delta_horiz * sin_deg(heading) + delta_vert * cos_deg(heading))

            # Store current odom positions for next loop
            self.prev_vert = curr_vert
            self.prev_horiz = curr_horiz

    def get_pos(self):
        return self.pos



if DriveTrainType == "tank":
    drive = TankDrivetrain(LeftMotors, RightMotors)
elif DriveTrainType == "holomonic":
    drive = HolomonicDrive(LeftMotors, RightMotors)
elif DriveTrainType == "holomonic6":
    drive = HolomonicDrive6(LeftMotors, RightMotors)
else:
    drive = None
    print("ded")

if AmtOdom == 0:
    Tracking = TrackingNoOdom
elif AmtOdom == 1:
    Tracking = Tracking1Odom
elif AmtOdom == 2:
    Tracking = Tracking2Odom
else:
    print("ded")
