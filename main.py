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
# This object initializes the ev3 brick
ev3 = EV3Brick()
# This object initializes the motor on port b
motor_b = Motor(Port.B)
# This object initializes the motor on port c
motor_c = Motor(Port.C)
# This object initializes the motor on port d
motor_d = Motor(Port.D)
# motor_d.run_angle(1560, 1560, then=Stop.HOLD, wait=True)
# This object initializes the motor on port a
motor_a = Motor(Port.A)
# This object initializes the drivebase which is the robot, we can use
# this to move more accurately straight, this takes in the left and right motors,
# as well as the diameter of our wheels and the distance each wheel is from each other in millimeters
robot = DriveBase(motor_b, motor_c, 94, 95)
# This object initializes the color sensor on port 3
color = ColorSensor(Port.S3)
# This object initializes the gyro sensor on port 2
# gyro = GyroSensor(Port.S2)
# This object initializes the ultrasonic sensor on port 4
# sonic = UltrasonicSensor(Port.S4)
# initialize function myblocks here

# This function is our moveTank block which takes in the
# speed, steering, and distance so that we can turn gradually


def gyroStraight(speed, distance):
    robot.reset()
    gyroint = gyro.angle()
    while robot.distance() <= distance:
        robot.drive(speed, -(gyro.angle() - gyroint))


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

# this is our line following function which takes in
# speed, proportianal gain, our robot object, and our distance


def LineFollow(speed, proportianal_gain, robot, distance):
    black = 9
    white = 100
    threshold = (black + white)/2
    robot.reset()
    while robot.distance() < distance:
        deviation = color.reflection() - threshold
        turn_rate = proportianal_gain * deviation
        robot.drive(speed, turn_rate)
    robot.stop(Stop.HOLD)

# this function stops the robot and gets rid of all momentum


def dead_stop():
    robot.stop()
    motor_b.hold()
    motor_c.hold()

# Write your program here.

# this function does step counter


def Step_counter(robot):
    robot.stop()
    robot.settings(1560, 1560, 200, 200)
    robot.straight(300)
    dead_stop()
    wait(1000)
    robot.straight(70)
    dead_stop()
    robot.turn(15)
    dead_stop()
    robot.stop()
    robot.settings(300, 300, 200, 200)
    robot.stop()
    robot.straight(500)
    dead_stop()
    robot.straight(-20)
    dead_stop()
    robot.straight(140)
    dead_stop()
    robot.stop()
    robot.settings(900, 900, 200, 200)
    # robot.straight(-500)
    moveTank(-1560, 30, -400)
    robot.turn(100)
    robot.drive_time(100, 0, 1000)
    robot.straight(-70)
    robot.stop()
    motor_b, motor_c = Motor(
        Port.B, positive_direction=Direction.COUNTERCLOCKWISE), Motor(
        Port.C, positive_direction=Direction.COUNTERCLOCKWISE)
    robot = DriveBase(motor_b, motor_c, wheel_diameter=94.2, axle_track=95)
    LineFollow(120, .5, robot, 750)
    # robot.turn(28)
    # robot.straight(150)
    # robot.turn(-32)
    # motor_d.run_time(-500, 1000, then=Stop.COAST, wait=False)
    # robot.straight(125)
    # wait(1000)
    # motor_d.run_time(-200, 9500, then=Stop.COAST, wait=True)
    # robot.straight(-200)
    # robot.turn(-210)
    # LineFollow(100, 1.0, robot, 400)
    # LineFollow(20, 1.0, robot, 50)
    # robot.straight(1000)
    return
    dead_stop()
    robot.stop()
    robot.settings(1560, -300, 200, 200)
    motor_a.run_angle(1560, -100, then=Stop.HOLD, wait=True)
    robot.turn(-40)
    dead_stop()
    robot.straight(200)
    dead_stop()
    robot.turn(40)
    dead_stop()
    motor_a.run_angle(1560, -100, then=Stop.HOLD, wait=False)
    robot.stop()
    robot.settings(1560, -500, 200, 200)

    robot.straight(720)
    dead_stop()
    robot.stop()
    robot.settings(200, 200, 200, 200)
    robot.turn(-90)
    robot.stop()
    robot.straight(-190)
    robot.stop()
    robot.settings(100, 100, 200, 200)
    robot.straight(170)
    robot.stop()
    robot.settings(900, 900, 100, 50)
    robot.turn(-90)
    motor_d.run_time(-200, 2000, then=Stop.COAST, wait=False)
    robot.straight(-60)
    wait(1000)
    motor_d.run_time(-200, 9500, then=Stop.COAST, wait=True)
    robot.stop()
    robot.settings(400, 400, 200, 200)
    robot.straight(110)
    robot.turn(100)
    robot.drive_time(-100, 0, 1500)
    robot.straight(200)
    robot.turn(50)
    motor_a.run_angle(1560, -200, then=Stop.HOLD, wait=False)
    robot.straight(75)
    motor_a.run_angle(1560, 400, then=Stop.HOLD, wait=True)
    robot.straight(-80)
    robot.turn(-60)
    robot.straight(60)
    motor_a.run_angle(1560, -300, then=Stop.HOLD, wait=True)
    robot.straight(-100)
    robot.turn(90)
    motor_a.run_angle(1560, 200, then=Stop.HOLD, wait=False)
    robot.stop()
    motor_b, motor_c = Motor(
        Port.B, positive_direction=Direction.COUNTERCLOCKWISE), Motor(
        Port.C, positive_direction=Direction.COUNTERCLOCKWISE)
    robot = DriveBase(motor_b, motor_c, wheel_diameter=94.2, axle_track=95)
    LineFollow(60, -.6, robot, 350)
    robot.turn(-5)
    robot.stop()
    robot.settings(900, 1560, 0, 0)
    robot.straight(1600)

# This function runs bench


def bench(robot):
    # robot.straight(-20)
    motor_a.run_angle(1560, -600, then=Stop.HOLD, wait=True)
    robot.straight(470)
    dead_stop()
    motor_a.run_angle(1560, -200, then=Stop.HOLD, wait=True)
    robot.straight(-90)
    dead_stop()
    robot.turn(-15)
    dead_stop()
    robot.straight(50)
    dead_stop()
    robot.straight(-350)
    dead_stop()
    robot.turn(-80)
    dead_stop()
    robot.straight(200)
    dead_stop()
    motor_a.run_angle(1560, 850, then=Stop.HOLD, wait=True)
# this function runs basketball, boccia, slide, and dance


def dropCubes():
    robot.stop()
    robot.settings(100, 100, 50, -50)
    robot.drive_time(-100, 0, 3)
    robot.straight(410)
    robot.stop()
    robot.settings(500, 500, 100, -100)
    robot.straight(-400)
    # robot.turn(15)
    # robot.straight(80)
    # robot.stop()
    # robot.settings(500, 500, 100, -100)
    # robot.straight(-400)


def bocciaketball(robot):
    robot.stop()
    motor_b, motor_c = Motor(
        Port.B, positive_direction=Direction.COUNTERCLOCKWISE), Motor(
        Port.C, positive_direction=Direction.COUNTERCLOCKWISE)
    robot = DriveBase(motor_b, motor_c, wheel_diameter=94.2, axle_track=95)
    robot.turn(-10)
    motor_a.run_angle(1560, -600, then=Stop.HOLD, wait=False)
    robot.straight(620)
    robot.turn(40)
    LineFollow(80, -.9, robot, 200)
    wait(500)
    while sonic.distance() < 570:
        robot.drive(-100, 0)
    robot.stop()
    # robot.turn(-120)


# This function is the logic and runs all of our mission function


def boccia2(robot):
    # Moves robot 1200mm to boccia2
    # robot.settings(600, 550, 50, 100)
    # robot.straight(150)
    # robot.turn(56)
    # dead_stop()
    robot.stop()
    robot.settings(100, 100, 50, 100)
    motor_a.run_angle(1560, -150, then=Stop.HOLD, wait=False)
    gyroStraight(100, 600)
    robot.stop()
    robot.straight(200)
    dead_stop()
    robot.stop()
    robot.settings(1560, 1560, 50, 100)
    robot.straight(200)
    dead_stop()
    motor_a.run_angle(1560, -200, then=Stop.HOLD, wait=False)
    robot.settings(200, 200, 100, 100)
    robot.straight(-50)
    robot.straight(-160)
    robot.turn(-135)
    robot.straight(-300)
    robot.turn(-40)
    robot.straight(150)
    motor_a.run_angle(1560, -900, then=Stop.HOLD, wait=False)
    robot.straight(130)
    robot.stop()
    robot.straight(-200)
    robot.turn(-120)
    while gyro.angle() != -270:
        gyrodata = gyro.angle()
        print(gyrodata)
        print(-270 - gyrodata)
        robot.turn(-270 - gyrodata)
        print(gyro.angle())
    return
    robot.straight(300)
    ultradata = sonic.distance()
    robot.straight(ultradata - 1234)
    robot.turn(90)
    motor_a.run_angle(1560, 1000, then=Stop.HOLD, wait=False)
    robot.drive_time(-100, 0, 3000)
    motor_a.run_angle(1560, -500, then=Stop.HOLD, wait=True)
    motor_a.run_angle(1560, 500, then=Stop.HOLD, wait=True)
    robot.straight(100)
    while True:
        motor_a.run_angle(1560, -500, then=Stop.HOLD, wait=True)
        motor_a.run_angle(1560, 500, then=Stop.HOLD, wait=True)
    robot.stop()
    # robot.straght(200)
    robot.stop()
    robot.turn(60)
    robot.straight(200)
    robot.turn(-95)
    robot.drive_time(-100, 0, 3000)
    motor_a.run_angle(1560, -500, then=Stop.HOLD, wait=True)
    motor_a.run_angle(1560, 500, then=Stop.HOLD, wait=True)
    robot.straight(100)
    while True:
        motor_a.run_angle(1560, -500, then=Stop.HOLD, wait=True)
        motor_a.run_angle(1560, 500, then=Stop.HOLD, wait=True)
    robot.stop()
    robot.straight(100)
    while True:
        motor_a.run_angle(1560, -500, then=Stop.HOLD, wait=True)
        motor_a.run_angle(1560, 500, then=Stop.HOLD, wait=True)
    robot.stop()
    motor_b, motor_c = Motor(
        Port.B, positive_direction=Direction.COUNTERCLOCKWISE), Motor(
        Port.C, positive_direction=Direction.COUNTERCLOCKWISE)
    robot = DriveBase(motor_b, motor_c, wheel_diameter=94.2, axle_track=95)
    LineFollow(100, -.8, robot, 200)
    robot.stop()
    motor_b, motor_c = Motor(
        Port.B, positive_direction=Direction.CLOCKWISE), Motor(
        Port.C, positive_direction=Direction.CLOCKWISE)
    robot = DriveBase(motor_b, motor_c, wheel_diameter=94.2, axle_track=95)
    robot.turn(15)
    robot.straight(100)
    robot.turn(90)
    robot.straight(100)
    motor_a.run_angle(1560, -1000, then=Stop.HOLD, wait=True)
    motor_a.run_angle(1560, 1000, then=Stop.HOLD, wait=False)
    robot.turn(-20)
    robot.straight(-200)
    robot.turn(-90)
    robot.straight(-200)
    while True:
        motor_a.run_angle(1560, -1000, then=Stop.HOLD, wait=True)
        motor_a.run_angle(1560, 1000, then=Stop.HOLD, wait=True)

    # robot.turn(120)
    # robot.straight(300)
    # robot.turn(10)
    # motor_a.run_angle(1560, -1000, then=Stop.HOLD, wait=True)
    # robot.straight(-200)
    # motor_a.run_an gle(1560, 1000, then=Stop.HOLD, wait=True)
    # robot.turn(120)
    # motor_a.run_angle(1560, -900, then=Stop.HOLD, wait=False)
    # robot.straight(400)
    # robot.turn(-10)
    # moveTank(-1560, 0, 500)


def main(robot):
    # boccia2(robot)
    # bocciaketball(robot)
    # robot.stop()
    # robot.settings(1560, -300, 100, 100)
    # while len(ev3.buttons.pressed()) == 0:
    #     pass
    # bench(robot)
    # while len(ev3.buttons.pressed()) == 0:
    #     pass
    # dropCubes()
    while len(ev3.buttons.pressed()) == 0:
        pass
    wait(500)
    robot.straight(-20)
    Step_counter(robot)
    robot.stop()
    robot.settings(200, 200, 100, 100)
    while len(ev3.buttons.pressed()) == 0:
        pass
    robot.straight(80)
    dead_stop()
    robot.turn(49)
    dead_stop()
    boccia2(robot)


# This runs our main function
main(robot)
# moveTank(-100, 10, -400)
# print(color.reflection())
# LineFollow(80, -.7, robot, 1000)
# robot.stop()
# motor_b.brake()
# motor_c.brake()
# motor_a.run_angle(1560, -1000, then=Stop.HOLD, wait=True)
# gyroFixed(90)
# gyroFixed(0)
# # robot.straight(300)
# robot.stop()
# robot.settings(1560, -400, 0, 0)
# robot.drive_time(-100, 0, 1000)
# robot.straight(500)
# robot.straight(-500)
# gyroStraight(100, 100)
# dead_stop()
# print(gyro.angle())
# robot.stop()
# robot.settings(400, 400, 100, 100)
# robot.drive_time(-100, 0, 1000)
# motor_a.run_angle(1560, -100, then=Stop.HOLD, wait=True)
# robot.straight(50)
# while len(ev3.buttons.pressed()) == 0:
#     pass
# robot.turn(35)
# robot.straight(850)
# motor_a.run_angle(1560, -500, then=Stop.HOLD, wait=True)
# motor_a.run_angle(1560, 500, then=Stop.HOLD, wait=True)
# robot.straight(90)
# while True:
#     motor_a.run_angle(1560, -1000, then=Stop.HOLD, wait=True)
#     motor_a.run_angle(1560, 1000, then=Stop.HOLD, wait=True)
# robot.straight(500)
