from venice import Direction, Gearset, Motor, RotationSensor, RotationUnit
from Classes import Point, factorial, sin_deg, cos_deg, deg_to_rad


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


def create_motor(port: int) -> Motor:
    port_abs: int = abs(port)
    if port < 0:
        return Motor(port_abs, Direction.REVERSE, Gearset.GREEN)
    else:
        return Motor(port_abs, Direction.FORWARD, Gearset.GREEN)


def create_rotation(port: int) -> RotationSensor:
    port_abs: int = abs(port)
    if port < 0:
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

        if velocity + turn > 12 or velocity + turn < -12:
            right_Norm = velocity + turn - 12
        elif velocity - turn > 12 or velocity - turn < -12:
            left_norm = velocity - turn -12

        right_norm = 0
        left_norm = 0

        for m in self.left_motors:
            m.set_voltage(velocity + turn - left_norm)
        for m in self.right_motors:
            m.set_voltage(velocity - turn - right_norm)

    def MotorPosition(self):
        Amt_Motors = 0
        Total = 0
        for m in self.left_motors + self.right_motors:
            Amt_Motors += 1
            Total += m.raw_position()
        return Total / Amt_Motors


class HolomonicDrive:
    def __init__(self, left_ports: list[int], right_ports: list[int]):
        self.LeftFront = create_motor(left_ports[0])
        self.LeftBack = create_motor(left_ports[1])
        self.RightFront = create_motor(right_ports[0])
        self.RightBack = create_motor(right_ports[1])

    def drive_tank(self, Forward, Rotate):

        right_norm = 0
        left_norm = 0

        if Forward + Rotate > 12 or Forward + Rotate < -12:
            right_Norm = Forward + Rotate - 12
        elif Forward - Rotate > 12 or Forward - Rotate < -12:
            left_norm = Forward - Rotate -12

        FL = max(-12, min(12, Forward + Rotate - left_norm))
        FR = max(-12, min(12, Forward - Rotate - right_norm))
        BL = max(-12, min(12, Forward + Rotate - left_norm))
        BR = max(-12, min(12, Forward - Rotate - right_norm))

        self.LeftFront.set_voltage(FL)
        self.RightFront.set_voltage(FR)
        self.LeftBack.set_voltage(BL)
        self.RightBack.set_voltage(BR)

    def drive_holomonic(self, Forward, Lateral, Rotate):
        FL = max(-12, min(12, Forward + Lateral + Rotate))
        BR = max(-12, min(12, Forward + Lateral - Rotate))

        self.LeftFront.set_voltage(FL)
        self.RightFront.set_voltage(FR)
        self.LeftBack.set_voltage(BL)
        self.RightBack.set_voltage(BR)

    def MotorPosition(self):
        Total = 0
        Total += self.LeftFront.raw_position()
        Total += self.RightFront.raw_position()
        Total += self.RightBack.raw_position()
        Total += self.LeftBack.raw_position()

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

        right_norm = 0
        left_norm = 0

        if Forward + Rotate > 12 or Forward + Rotate < -12:
            right_Norm = Forward + Rotate - 12
        elif Forward - Rotate > 12 or Forward - Rotate < -12:
            left_norm = Forward - Rotate -12



        FL = max(-12, min(12, Forward + Rotate - left_norm))
        FR = max(-12, min(12, Forward - Rotate - right_norm))
        ML = max(-12, min(12, Forward + Rotate - left_norm))
        MR = max(-12, min(12, Forward - Rotate - right_norm))
        BL = max(-12, min(12, Forward + Rotate - left_norm))
        BR = max(-12, min(12, Forward - Rotate - right_norm))

        self.LeftFront.set_voltage(FL)
        self.RightFront.set_voltage(FR)
        self.LeftMiddle.set_voltage(ML)
        self.RightMiddle.set_voltage(MR)
        self.LeftBack.set_voltage(BL)
        self.RightBack.set_voltage(BR)

    def drive_holomonic(self, Forward, Lateral, Rotate):
        FL = max(-12, min(12, Forward + Lateral + Rotate))
        FR = max(-12, min(12, Forward - Lateral - Rotate))
        ML = max(-12, min(12, Forward + Rotate))
        MR = max(-12, min(12, Forward - Rotate))
        BL = max(-12, min(12, Forward - Lateral + Rotate))
        BR = max(-12, min(12, Forward + Lateral - Rotate))

        self.LeftFront.set_voltage(FL)
        self.RightFront.set_voltage(FR)
        self.LeftMiddle.set_voltage(ML)
        self.RightMiddle.set_voltage(MR)
        self.LeftBack.set_voltage(BL)
        self.RightBack.set_voltage(BR)

    def MotorPosition(self):
        Total = 0
        Total += self.LeftFront.raw_position()
        Total += self.RightFront.raw_position()
        Total += self.RightMiddle.raw_position()
        Total += self.LeftMiddle.raw_position()
        Total += self.RightBack.raw_position()
        Total += self.LeftBack.raw_position()

        return Total / 6

class TrackingNoOdom:
    def __init__(self, Gyro, drive, GearRatio, WheelDiameter):
        self.gyro = Gyro
        self.drive = drive
        self.pos = Point(0, 0)
        self.GearRatio = GearRatio
        self.WheelDiameter = WheelDiameter

        # Store previous motor position to calculate delta
        self.prev_motor = self.drive.MotorPosition

    # This function continuously updates the position of the robot
    # Each loop it takes the information from the motors and updates the positions
    def updatePose(self):
        while True:
            # Read current motor rotation
            curr_motor = self.drive.MotorPosition

            # Calculate change since last update
            delta_motor = curr_motor - self.prev_motor
            Wheel_rotations = (delta_motor / 4096) * self.GearRatio
            DistanceMoved = Wheel_rotations * 3.14159 * self.WheelDiameter

            # Update global position using current heading
            heading = self.gyro.get_heading
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
        self.OdomPod1 = create_rotation(OdomPorts)

        # Store previous pod position to calculate delta
        self.prev_odom = self.OdomPod1.position(DEGREES)

    # This function continuously updates the position of the robot
    # Each loop it takes the information from the odom pod and updates the positions
    def updatePose(self):
        while True:
            # Read current odom pod rotation
            curr_odom = self.OdomPod1.position(DEGREES)

            # Calculate change since last update
            delta_rot = curr_odom - self.prev_odom
            DistanceMoved = delta_rot * self.OdomDiameter * 3.14159 / 360  # convert degrees to revolutions

            # Update global position using current heading
            heading = self.gyro.get_heading
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
        self.prev_vert = self.OdomPodVertical.position(DEGREES)
        self.prev_horiz = self.OdomPodHorizontal.position(DEGREES)

    # This function continuously updates the position of the robot
    # Each loop it takes the information from the odom pods and updates the positions
    def updatePose(self):
        while True:

            # Read current odom pod rotations
            curr_vert = self.OdomPodVertical.position(DEGREES)
            curr_horiz = self.OdomPodHorizontal.position(DEGREES)

            # Compute how much each pod moved since last update
            delta_vert = (curr_vert - self.prev_vert) * self.OdomDiameter * 3.14159 / 360
            delta_horiz = (curr_horiz - self.prev_horiz) * self.OdomDiameter * 3.14159 / 360

            # Rotate local movement by current heading to get field coordinates
            heading = self.gyro.get_heading
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
    print("ded")

if AmtOdom == 0:
    Tracking = TrackingNoOdom
elif AmtOdom == 1:
    Tracking = Tracking1Odom
elif AmtOdom == 2:
    Tracking = Tracking2Odom
else:
    print("ded")
