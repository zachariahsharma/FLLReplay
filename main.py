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
    moveTank(1500, 20, 825)
    dead_stop()
    wait(500)
    for i in range(0, 5):
        robot.drive_time(25, 0, 2490)
        robot.stop(Stop.BRAKE)
        robot.drive_time(-100, 0, 300)
        robot.stop(Stop.HOLD)


def Treadmill(robot):
    moveTank(-300, 95, -130)
    robot.turn(115)
    robot.stop()
    motor_b, motor_c = Motor(
        Port.B, positive_direction=Direction.COUNTERCLOCKWISE), Motor(
        Port.C, positive_direction=Direction.COUNTERCLOCKWISE)
    robot = DriveBase(motor_b, motor_c, wheel_diameter=94.2, axle_track=95)
    LineFollow(60, 1.05, robot, 520)
    robot.turn(28)
    robot.straight(140)
    robot.turn(-33)
    motor_d.run_time(-500, 1000, then=Stop.COAST, wait=False)
    robot.straight(120)
    wait(1000)
    motor_d.run_time(-200, 10000, then=Stop.COAST, wait=True)


def slide(robot):


def bench(robot):
    robot.straight(400)
    robot.turn(-20)
    robot.turn(20)
    robot.straight(-50)
    robot.turn(-10)
    robot.straight(150)
    motor_a.run_angle(1560, -600, then=Stop.HOLD, wait=True)
    robot.turn(30)
    motor_a.run_angle(1560, 600, then=Stop.HOLD, wait=False)
    robot.straight(-500)
    # motor_a.run_angle(1560, -100, then=Stop.HOLD, wait=True)


def main(robot):
    bench(robot)
    while len(ev3.buttons.pressed) = 0:
        pass
    wait(1000)
    gyro.reset_angle(0)
    robot.drive_time(-100, 0, 1600)
    Step_counter()
    Treadmill(robot)


main(robot)
