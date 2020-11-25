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
motor_b = Motor(Port.B)
motor_c = Motor(Port.C)
motor_d = Motor(Port.D)
motor_a = Motor(Port.A)
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
    for i in range(0, 6):
        robot.drive_time(25, 0, 2490)
        robot.stop(Stop.BRAKE)
        robot.drive_time(-100, 0, 300)
        robot.stop(Stop.HOLD)


def Treadmill(robot):
    moveTank(-300, 95, -130)
    robot.turn(112)
    robot.stop()
    motor_b, motor_c = Motor(
        Port.B, positive_direction=Direction.COUNTERCLOCKWISE), Motor(
        Port.C, positive_direction=Direction.COUNTERCLOCKWISE)
    robot = DriveBase(motor_b, motor_c, wheel_diameter=94.2, axle_track=95)
    LineFollow(60, 1.05, robot, 520)
    robot.turn(28)
    robot.straight(140)
    robot.turn(-32)
    motor_d.run_time(-500, 1000, then=Stop.COAST, wait=False)
    robot.straight(120)
    wait(1000)
    motor_d.run_time(-200, 9500, then=Stop.COAST, wait=True)


def bench(robot):
    robot.straight(400)
    robot.turn(-20)
    robot.turn(20)
    robot.straight(-50)
    robot.turn(-20)
    robot.straight(170)
    motor_a.run_angle(1560, -600, then=Stop.HOLD, wait=True)
    robot.turn(35)
    motor_a.run_angle(1560, 700, then=Stop.HOLD, wait=False)
    robot.straight(-500)
    # motor_a.run_angle(1560, -100, then=Stop.HOLD, wait=True)


def bocciaketballslide():
    motor_b, motor_c = Motor(
        Port.B, positive_direction=Direction.COUNTERCLOCKWISE), Motor(
        Port.C, positive_direction=Direction.COUNTERCLOCKWISE)
    robot = DriveBase(motor_b, motor_c, wheel_diameter=94.2, axle_track=95)
    robot.straight(50)
    motor_a.run_angle(1560, -180, then=Stop.HOLD, wait=False)
    LineFollow(60, -1.2, robot, 750)
    robot.turn(-170)
    robot.straight(-160)
    motor_a.run_angle(1560, -1100, then=Stop.HOLD, wait=True)
    motor_a.run_angle(1560, 900, then=Stop.HOLD, wait=True)
    robot.straight(100)
    robot.turn(-20)
    robot.straight(-40)
    motor_a.run_angle(1560, -900, then=Stop.HOLD, wait=True)
    motor_a.run_angle(1560, 1150, then=Stop.HOLD, wait=True)
    # robot.straight(200)
    # robot.turn(60)
    # LineFollow(50, 1.2, robot, 450)
    # robot.turn(40)
    # robot.straight(-100)
    # motor_a.run_angle(1560, -1000, then=Stop.HOLD, wait=False)
    # robot.straight(-100)
    # wait(2000)
    # robot.straight(-10)


def main(robot):
    bocciaketballslide()
    # motor_a.run_angle(1560, -150, then=Stop.HOLD, wait=True)
    # while len(ev3.buttons.pressed()) == 0:
    #     pass
    # bench(robot)
    # while len(ev3.buttons.pressed()) == 0:
    #     pass
    # robot.straight(700)
    # robot.straight(-1000)
    # while len(ev3.buttons.pressed()) == 0:
    #     pass
    # wait(500)
    # gyro.reset_angle(0)
    # robot.drive_time(-100, 0, 1000)
    # Step_counter()
    # Treadmill(robot)


main(robot)
