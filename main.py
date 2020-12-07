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
# This object initializes the motor on port a
motor_a = Motor(Port.A)
# This object initializes the drivebase which is the robot, we can use
# this to move more accurately straight, this takes in the left and right motors,
# as well as the diameter of our wheels and the distance each wheel is from each other in millimeters
robot = DriveBase(motor_b, motor_c, wheel_diameter=94, axle_track=95)
# This object initializes the color sensor on port 3
color = ColorSensor(Port.S3)
# This object initializes the gyro sensor on port 2
gyro = GyroSensor(Port.S2)
# This object initializes the ultrasonic sensor on port 4
sonic = UltrasonicSensor(Port.S4)
# initialize function myblocks here

# This function is our moveTank block which takes in the 
# speed, steering, and distance so that we can turn gradually
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

# this is our line following function which takes in speed, proportianal gain, our robot object, and our distance
def LineFollow(speed, proportianal_gain, robot, distance):
    black = 5
    white = 53
    threshold = (black + white)/2
    robot.reset()
    while robot.distance() < distance:
        deviation = color.reflection() - threshold
        turn_rate = proportianal_gain * deviation
        robot.drive(speed, turn_rate)
    robot.stop(Stop.HOLD)

# this function  
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
    moveTank(-300, 120, -160)
    robot.turn(100)
    robot.straight(-70)
    robot.stop()
    motor_b, motor_c = Motor(
        Port.B, positive_direction=Direction.COUNTERCLOCKWISE), Motor(
        Port.C, positive_direction=Direction.COUNTERCLOCKWISE)
    robot = DriveBase(motor_b, motor_c, wheel_diameter=94.2, axle_track=95)
    LineFollow(60, 1.05, robot, 455)
    robot.turn(25)
    robot.straight(140)
    robot.turn(-32)
    motor_d.run_time(-500, 1000, then=Stop.COAST, wait=False)
    robot.straight(115)
    wait(1000)
    motor_d.run_time(-200, 9500, then=Stop.COAST, wait=True)
    robot.straight(-200)
    robot.turn(-240)
    LineFollow(60, -1.0, robot, 400)
    robot.turn(5)
    moveTank(1560, 0, 1100)


def bench(robot):
    robot.straight(400)
    robot.turn(-20)
    robot.turn(20)
    robot.straight(-100)
    robot.turn(-60)
    robot.straight(162.5)
    robot.turn(60)
    robot.straight(92.5)
    motor_a.run_angle(1560, -600, then=Stop.HOLD, wait=True)
    robot.turn(-20)
    robot.turn(20)
    robot.turn(50)
    motor_a.run_angle(1560, 800, then=Stop.HOLD, wait=False)
    robot.straight(-500)


def bocciaketball(robot):
    robot.stop()
    motor_b, motor_c = Motor(
        Port.B, positive_direction=Direction.COUNTERCLOCKWISE), Motor(
        Port.C, positive_direction=Direction.COUNTERCLOCKWISE)
    robot = DriveBase(motor_b, motor_c, wheel_diameter=94.2, axle_track=95)
    robot.straight(50)
    motor_a.run_angle(1560, -250, then=Stop.HOLD, wait=False)
    LineFollow(50, -1.2, robot, 700)
    robot.turn(-145)
    robot.straight(-195)
    motor_a.run_angle(1560, -950, then=Stop.HOLD, wait=True)
    motor_a.run_angle(1560, 900, then=Stop.HOLD, wait=False)
    robot.straight(200)
    robot.turn(60)
    LineFollow(50, 1.1, robot, 450)
    robot.turn(30)
    robot.straight(-100)
    motor_a.run_angle(1560, -1000, then=Stop.HOLD, wait=False)
    robot.straight(-100)
    wait(2000)
    robot.straight(-50)
    wait(1000)
    robot.straight(200)
    robot.turn(-80)
    robot.straight(-300)
    robot.turn(80)
    while True:
        motor_a.run_angle(1560, 1000, then=Stop.HOLD, wait=True)
        motor_a.run_angle(1560, -1000, then=Stop.HOLD, wait=True)


def main(robot):
    motor_a.run_angle(1560, -200, then=Stop.HOLD, wait=True)
    while len(ev3.buttons.pressed()) == 0:
        pass
    bench(robot)
    while len(ev3.buttons.pressed()) == 0:
        pass
    motor_a.run_angle(1560, -200, then=Stop.HOLD, wait=True)
    wait(500)
    gyro.reset_angle(0)
    robot.drive_time(-100, 0, 1000)
    Step_counter()
    Treadmill(robot)
    motor_a.run_angle(1560, 300, then=Stop.HOLD, wait=True)
    while len(ev3.buttons.pressed()) == 0:
        pass
    bocciaketball(robot)


main(robot)
