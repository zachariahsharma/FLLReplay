#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# Create your objects here.
ev3 = EV3Brick()
motor_b, motor_c, motor_d, motor_a = Motor(
    Port.B), Motor(Port.C), Motor(Port.D), Motor(Port.A)
robot = DriveBase(motor_b, motor_c, wheel_diameter=94.2, axle_track=95)
color = ColorSensor(Port.S3)
gyro = GyroSensor(Port.S2)
sonic = UltrasonicSensor(Port.S4)
# initialize function myblocks here


def moveTank(speed, steering, distance):
    robot.reset()
    robot.drive(speed, steering)
    if distance < 0:
        while robot.distance() > distance:
            pass
    else:
        while robot.distance() < distance:
            pass

    robot.stop(Stop.HOLD)


def LineFollow(speed, proportianal_gain, robot, distance):
    black = 11
    white = 90
    threshold = (black + white)/2
    robot.reset()
    while robot.distance() < distance:
        deviation = color.reflection() - threshold
        turn_rate = proportianal_gain * deviation
        robot.drive(speed, turn_rate)
    robot.stop(Stop.HOLD)


# Write your program here.


def Step_counter():
    ev3.speaker.beep()
    moveTank(300, 30, 950)
    wait(500)
    for i in range(0, 2):
        robot.drive_time(50, 20, 2100)
        robot.stop(Stop.COAST)
        robot.drive_time(-80, 0, 200)
        robot.stop(Stop.HOLD)


def Treadmill(robot):
    moveTank(-300, 110, -150)
    robot.turn(120)
    robot.stop()
    motor_b, motor_c = Motor(
        Port.B, positive_direction=Direction.COUNTERCLOCKWISE), Motor(
        Port.C, positive_direction=Direction.COUNTERCLOCKWISE)
    robot = DriveBase(motor_b, motor_c, wheel_diameter=94.2, axle_track=95)
    LineFollow(100, 1.05, robot, 500)
    ev3.speaker.beep()
    robot.turn(20)
    robot.straight(130)
    robot.turn(-20) hey bob
    motor_d.run_time(-500, 1000, then=Stop.COAST, wait=False)
    robot.straight(160)
    wait(1000)
    motor_d.run_time(-200, 10000, then=Stop.COAST, wait=True)
    robot.stop()


def weightMachine():
    motor_b, motor_c = Motor(
        Port.B, positive_direction=Direction.CLOCKWISE), Motor(
        Port.C, positive_direction=Direction.CLOCKWISE)
    robot = DriveBase(motor_b, motor_c, wheel_diameter=94.2, axle_track=95)
    robot.straight(200)
    wait(500)
    # robot.stop()
    print(gyro.angle())
    ev3.speaker.beep()
    while gyro.angle() < 260:
        robot.drive(-100, 100)
    robot.stop()
    robot.straight(300)
    robot.stop()
    while sonic.distance() > 320:
        robot.drive(100, 0)


if __name__ == "__main__":
    motor_a.run_angle(1560, -1300, then=Stop.HOLD, wait=False)
    gyro.reset_angle(0)
    print(gyro.angle())
    robot.drive_time(-100, 0, 1600)
    Step_counter()
    Treadmill(robot)
    weightMachine()
