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


def dead_stop():
    robot.stop()
    motor_b.hold()
    motor_c.hold()

# Write your program here.


def Step_counter():
    moveTank(1500, 15, 825)
    dead_stop()
    wait(500)
    for i in range(0, 5):
        robot.drive_time(25, 0, 2400)
        robot.stop(Stop.COAST)
        robot.drive_time(-80, 0, 200)
        robot.stop(Stop.HOLD)


def Treadmill(robot):
    print('my name is bob')
    moveTank(-300, 95, -130)
    robot.turn(105)
    robot.stop()
    motor_b, motor_c = Motor(
        Port.B, positive_direction=Direction.COUNTERCLOCKWISE), Motor(
        Port.C, positive_direction=Direction.COUNTERCLOCKWISE)
    robot = DriveBase(motor_b, motor_c, wheel_diameter=94.2, axle_track=95)
    LineFollow(70, 1.05, robot, 550)
    robot.turn(28)
    robot.straight(180)
    robot.turn(-28)
    motor_d.run_time(-500, 1000, then=Stop.COAST, wait=False)
    robot.straight(70)
    wait(1000)
    motor_d.run_time(-200, 10000, then=Stop.COAST, wait=True)
    robot.stop()


def slide(robot):
    robot.stop()
    motor_b, motor_c = Motor(
        Port.B, positive_direction=Direction.CLOCKWISE), Motor(
        Port.C, positive_direction=Direction.CLOCKWISE)
    robot = DriveBase(motor_b, motor_c, wheel_diameter=94.2, axle_track=95)
    robot.straight(220)
    wait(500)
    # robot.stop()
    while gyro.angle() < 265:
        robot.drive(-100, 80)
    dead_stop()
    motor_a.run_angle(1500, -1000, then=Stop.HOLD, wait=False)
    robot.straight(515)
    dead_stop()
    robot.turn(125)  # 105
    robot.stop()
    motor_b, motor_c = Motor(
        Port.B, positive_direction=Direction.COUNTERCLOCKWISE), Motor(
        Port.C, positive_direction=Direction.COUNTERCLOCKWISE)
    robot = DriveBase(motor_b, motor_c, wheel_diameter=94.2, axle_track=95)
    robot.straight(200)
    robot.turn(70)
    robot.turn(-60)
    LineFollow(100, 1.05, robot, 50)
    robot.straight(250)


def main():
    # motor_a.run_angle(1560, -1300, then=Stop.HOLD, wait=False)
    gyro.reset_angle(0)
    print(gyro.angle())
    robot.drive_time(-100, 0, 1600)
    Step_counter()
    Treadmill(robot)
    slide(robot)
    # motor_a.run_angle(1500, 1000, then=Stop.HOLD, wait=True)
