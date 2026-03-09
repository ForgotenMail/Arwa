from math import degrees
from turtle import heading
from drivetrains import drive, Gyro, GearRatio, WheelDiamater

#This File controls all of the motion algorithms
#This file includes PID, and Pure Pursuit


LinearKp = 5
LinearKi = 5
LinearKd = 5

AngularKp = 5
AngularKi = 5
AngularKd = 5

def LinearPID(Distance, Speed, Target):

    time = brain.gettime()
    error = 0
    integral = 0
    heading = Gyro.get_heading(DEGREES)
    Driven = 0
    while Driven < Distance:
        MotorPose = drive.MotorPosition()

        deltaTime = brain.gettime() - time
        pasterror = error
        error = Target - heading

        P = LinearKp  * error
        integral += error * deltaTime
        Ipid = LinearKi * integral
        derivative = (error - pasterror) / deltaTime
        D = LinearKd * derivative

        PID = P + Ipid + D

        drive.drive_tank(Speed, PID)
        MotorDelta = MotorPose - drive.MotorPosition()
        Revolutions = (MotorDelta/4096) * GearRatio
        Driven = (WheelDiamater*3.14159)*Revolutions

def AngularPID(Speed, Target, Buffer)
    time = brain.gettime()
    error = 0
    integral = 0
    heading = Gyro.get_heading(DEGREES)
    while heading + Buffer < Target or heading - Buffer > Target:
        MotorPose = drive.MotorPosition()

        deltaTime = brain.gettime() - time
        pasterror = error
        error = Target - heading

        P = LinearKp  * error
        integral += error * deltaTime
        Ipid = LinearKi * integral
        derivative = (error - pasterror) / deltaTime
        D = LinearKd * derivative


        PID = P + Ipid + D

        drive.drive_tank(0, (Speed*PID)/12)
