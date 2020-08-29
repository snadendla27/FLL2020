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

# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
#medium_motor = Motor(Port.A)
front_largemotor = Motor(Port.D)
wheel_diameter = 56
axle_track = 115

# Initialize the color sensor.
line_sensor = ColorSensor(Port.S3)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)
robot.straight(200)
#robot.run_target(200, 0, then=Stop.HOLD, wait=True)

#this code stops the motor and actively holds it at its current angle
hold()

#this code turns the bot 90 degrees 
turn(90)

# Initialize the color sensor.
line_sensor = ColorSensor(Port.S3)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)
robot.straight(200)
#robot.run_target(200, 0, then=Stop.HOLD, wait=True)

#this code stops the motor and actively holds it at its current angle
hold()

#this code turns the bot 90 degrees 
turn(90)

# Initialize the color sensor.
line_sensor = ColorSensor(Port.S3)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)
robot.straight(200)
#robot.run_target(200, 0, then=Stop.HOLD, wait=True)

#Stops the motor and lets it spin freely. The motor gradually stops due to friction.
stop()



