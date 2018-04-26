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
from std_msgs.msg import String
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState



# The function is the "callback" that handles the messages as they come in: 
def callback(msg):
    motor_command =  msg.data #get motor angle command
    print("I got motor command: "+str(motor_command))

    motor_state = JointState()
    motor_state.current_pos = motor_command
    motor_state.error = 0.0
    pub.publish(motor_state)
    
    rate = rospy.Rate(1)
    rate.sleep()

rospy.init_node('Stepper_Driver')
sub = rospy.Subscriber('/tilt_controller/command', Float64, callback)
pub = rospy.Publisher('/tilt_controller/state', JointState, queue_size=10)


def main():
    rospy.spin()

if __name__ == '__main__':
    main()
