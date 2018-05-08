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
import serial


port = rospy.get_param('port', '/dev/ttyUSB3') 
baud = rospy.get_param('baud', 115200) 
ser = serial.Serial(port,baud); 

MICRO_STEPPING = 1
FULL_STEPS = 200.0
STEPS_PER_DEGREE = float(360.0/(FULL_STEPS*MICRO_STEPPING)) #convert degree to steps 

def moveMotor(goal_rad):
    if motor_state.is_moving: return

    #convert rad to degree

    #given abs goal_rad, we need compute error from current_pos to goal
    error_rad = goal_rad - motor_state.current_pos

    #publish a motor state message before move
    motor_state.error = error_rad
    motor_state.goal_pos = goal_rad 
    motor_state.is_moving = True
    pub.publish(motor_state)

    ser.flushInput()
    ser.flushOutput()
    ser.write("%4.4f" %(goal_rad));
    time.sleep(.1)


    motor_finished = False
    while True:
        try:
            result = ser.readline().splitlines()[0]
            #print("Get Result : %s" % result) 
            res = result.split(",")
            #print(res)
            #if message is not intact, skip
            if len(res) != 4:
                continue
            current_pos = float(res[0])
            error       = float(res[1])
            goal_pos    = float(res[2])
            is_moving   = bool(res[3])

            motor_state.current_pos = current_pos
            motor_state.error = error
            motor_state.goal_pos = goal_rad #use input, instead from arduino
            motor_state.is_moving = is_moving
            #pub.publish(motor_state)
            #print("Remaining Degree: %f" % np.rad2deg(error))

            if(motor_state.error == 0 or not motor_state.is_moving): 
                motor_finished = True
                break
        except Exception as e:
            print("Read Error:"+str(result))
            print(e)
            #break
        time.sleep(0.001)

    #after move
    #motor_state.error = 0.0
    #motor_state.current_pos = goal_rad 
    #motor_state.goal_pos = goal_rad 
    motor_state.is_moving = False 
    #pub.publish(motor_state)



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

rate = rospy.Rate(50) #10Hz

def main():
    

    while not rospy.is_shutdown():
        pub.publish(motor_state)
        rate.sleep()

if __name__ == '__main__':
    main()
