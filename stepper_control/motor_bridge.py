#!/usr/bin/env python
"""

    This program will subscribe /tilt_controller/command to move stepper motor 
    and publish /tilt_controller/state
    in /tilt_controller/state {current_pos: MOTOR_ANG_RAD_FLOAT, error: ERROR_BTN_GOAL_CURR_FLOAT}
  _____   _______   _____    _____ 
 |_   _| |__   __| |  __ \  |_   _|
   | |      | |    | |__) |   | |  
   | |      | |    |  _  /    | |  
  _| |_     | |    | | \ \   _| |_ 
 |_____|    |_|    |_|  \_\ |_____|

 Copyright (c) 2018 Industrial Technology Research Institute.
 195, Sec.4, Chung Hsing Rd., Chutung, Hsinchu, Taiwan 31057, R.O.C.
 All rights reserved.                 
                                      
 This software is the confidential and proprietary information of ITRI.
 ("Confidential Information").  You shall not disclose such Confidential
 Information and shall use it only in accordance with the terms of the license
 agreement you entered into with ITRI.   
 
 Primary maintainer: Kai Chang <kaichang@itri.org.tw>
"""
import time
import os
import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState


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
GPIO.output(ENABLE, False) #/EN low enable

step_speed = 1000
step_pin = GPIO.PWM(STEP,step_speed)

MICRO_STEPPING = 16
FULL_STEPS = 200.0
STEPS_PER_DEGREE = float(360.0/(FULL_STEPS*MICRO_STEPPING)) #convert degree to steps 

def moveMotor(goal_rad):
    if motor_state.is_moving:
        return
    motor_state.is_moving = True
    #convert rad to degree
    goal_degree = np.rad2deg(goal_rad)

    #given abs goal_rad, we need compute error from current_pos to goal
    error_rad = goal_rad - motor_state.current_pos

    print("Error Rad %f"%error_rad)
    #publish a motor state message before move
    motor_state.error = error_rad
    motor_state.goal_pos = goal_rad 
    #pub.publish(motor_state)


    error_degree = np.rad2deg(error_rad)
    goal_steps = int(error_degree / STEPS_PER_DEGREE)

    print("Error Deg %f"%error_degree)
    print("goal_steps %d"%goal_steps)
    #check direction 
    dir_sign = 1
    if goal_steps < 0:
        GPIO.output(DIR, False)
        goal_steps = -goal_steps
        dir_sign = -1
        print("Reverse:goal_steps %d"%goal_steps)
    else:
        GPIO.output(DIR, True)
        print("Forward:goal_steps %d"%goal_steps)



    steps = 0
    step_pin.ChangeFrequency(step_speed)
    while steps < int(goal_steps/10):
        #print("Move start steps %d, goal %d"%(steps,goal_steps))
        step_pin.start(1)
        #GPIO.output(STEP, True)
        #time.sleep(0.01)#100Hz
        #GPIO.output(STEP, False)

        steps += 1
        if steps % 10 == 0: #publish message every 10 steps
            remaining_steps = (goal_steps - steps)
            moved_rad = np.deg2rad(steps*STEPS_PER_DEGREE)
            new_error_degree = remaining_steps*STEPS_PER_DEGREE
            new_error_rad = np.deg2rad(new_error_degree*10)

            motor_state.current_pos += np.deg2rad(10*STEPS_PER_DEGREE)*dir_sign*10
            current_pos_deg = np.rad2deg(motor_state.current_pos)
            motor_state.error = new_error_rad
            #motor_state.goal_pos = goal_rad 
            #motor_state.is_moving = True
            #pub.publish(motor_state)
            print("Remaining Steps: %d,Goal %4.2f,Curr %4.2f" % (remaining_steps,goal_degree,current_pos_deg))
        time.sleep(0.01)#100Hz
    #after move
    motor_state.error = 0.0
    motor_state.current_pos = goal_rad 
    motor_state.goal_pos = goal_rad 
    motor_state.is_moving = False 
    #pub.publish(motor_state)

    step_pin.stop()


# The function is the "callback" that handles the messages as they come in: 
def callback(msg):
    motor_command =  msg.data #get motor angle command
    print("I got motor command: "+str(motor_command))

    moveMotor(motor_command)
    #pub.publish(motor_state)
    

rospy.init_node('Stepper_Driver')
motor_state = JointState()
motor_state.current_pos = 0.0
motor_state.error = 0.0
motor_state.goal_pos = 0.0
motor_state.is_moving = False 

sub = rospy.Subscriber('/tilt_controller/command', Float64, callback)
pub = rospy.Publisher('/tilt_controller/state', JointState, queue_size=10)

rate = rospy.Rate(10) #10Hz

def main():
    print("Start Stepper Node")
    while not rospy.is_shutdown():
        pub.publish(motor_state)
        rate.sleep()

if __name__ == '__main__':
    main()
