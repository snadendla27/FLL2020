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
left_motor = Motor(Port.C)
right_motor = Motor(Port.B)
medium_motor = Motor(Port.A)
front_largeMotor = Motor(Port.D)
wheel_diameter = 56
axle_track = 115

robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)
# Initialize the color sensor.
line_sensor = ColorSensor(Port.S2)

robot.straight(110)

# Calculate the light threshold. Choose values based on your measurements.
BLACK = 9
WHITE = 85
threshold = (BLACK + WHITE) / 2

# Set the drive speed at 100 millimeters per second.
DRIVE_SPEED = 100

# Set the gain of the proportional line controller. This means that for every
# percentage point of light deviating from the threshold, we set the turn
# rate of the drivebase to 1.2 degrees per second.

# For example, if the light value deviates from the threshold by 10, the robot
# steers at 10*1.2 = 12 degrees per second.
PROPORTIONAL_GAIN = 1.2
runWhile = True
# Start following the line endlessly.
while runWhile:
    # Calculate the deviation from the threshold.
    deviation = line_sensor.reflection() - threshold

    # Calculate the turn rate.
    turn_rate = PROPORTIONAL_GAIN * deviation

    # Set the drive base speed and turn rate.
    robot.drive(DRIVE_SPEED, turn_rate)

    if robot.distance() == 1400: 
        runWhile = False

# robot stops after finishing up line following code
robot.stop(Stop.BRAKE)

# robot turns after finishing up line following code
robot.turn(-103.5)

# robot goes straight as it heads towards the mission
robot.straight(138)

# robot turns right for 95 degrees
robot.turn(80)

# robot goes straight towards the mission to line the attachment to the wheel
robot.straight(90)

# large motor attachment goes down to trap the wheel in
front_largeMotor.run_angle(60, 160)

# robot moves backwards to bring wheel outside of the large circle
robot.straight (-100)

# large motor releases the trapped tire
front_largeMotor.run_angle(60, -148)

# robot moves straight to get closer the wheel
robot.straight (40)

# robot turns so the wheel can get into the smaller target
robot.turn (-40)

robot.stop (Stop.BRAKE)

# robot goes backwards to leave the target and the wheel inside of it
robot.straight (-70)

# robot turns towards the weight machine
robot.turn (-40)
