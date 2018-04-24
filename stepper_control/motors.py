#!/usr/bin/env python
#****************************************************************************
#  Project: How to manage a stepper motor via ROS?
#
#  Description:  The Subscriber Node for system with 12V Stepper motor. Receive the data. Starting the motor in the specified direction.
#  -------------------------------------------------------------------------
#****************************************************************************
import time
import os
import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
# Set up the pins what will need for us
DIR = 20
STEP = 21
ENABLE = 16
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)
GPIO.setup(ENABLE, GPIO.OUT)
GPIO.output(ENABLE, False)

step_speed = 500
p = GPIO.PWM(STEP,step_speed)

# The function is the "callback" that handles the messages as they come in: 
def callback(msg):
    print msg.data
    cmd = msg.data.split(',')


    if cmd[0] == 'F':
        # Send to the direction pin the True value
        GPIO.output(DIR, True)
    elif cmd[0] == 'R':
        # Send to the direction pin the False value
        GPIO.output(DIR, False)

    if int(cmd[2]) >= 100 and int(cmd[2]) <= 2000:
        step_speed = int(cmd[2])
        p.ChangeFrequency(step_speed)


    steps = 0
    # 1000 is the 2.5 rotation for this stepper(12 V)
    while steps < int(cmd[1]):
        p.start(1)
        steps += 1
        time.sleep(0.01)
    p.stop()
	
rospy.init_node('Stepper_Subscriber')
sub = rospy.Subscriber('rotation', String, callback)
rospy.spin()
