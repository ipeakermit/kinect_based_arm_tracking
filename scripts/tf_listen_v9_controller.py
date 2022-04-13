#!/usr/bin/env python
"""
Copyright 2019 VXLab
   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at
       http://www.apache.org/licenses/LICENSE-2.0
   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
   
   Originally forked from: https://github.com/seanshi007/kinect_based_arm_tracking
   Modified by: Lachlan Clulow
   Last Modified: 3/6/19
"""
import roslib
#roslib.load_manifest('kinect_based_arm_tracking')
import rospy
import math
import tf
import geometry_msgs.msg
import math
import baxter_interface
from std_msgs.msg import Float64
from baxter_interface import CHECK_VERSION
from baxter_core_msgs.msg import HeadPanCommand

import operator

# shoulder_a is the shoulder of the intested arm

'''
    This script moves a Baxter robot to mimic transforms for a tracked user.
    Sets a subscriber for each joint and when a subscriber receives a message it sets a corresponding
    element in the "command" dictionary, the main function also updates baxters arm joints with data from the
    command dictionary to corresponding joints every 1/10th of a second
'''

command = {"left": [0.0]*4, "right": [0.0]*4, "head": [0.0]*2}


def call_back_gen(limb_name, index):
    global command

    def call_back_func(f):
        global command
        # print limb_name
        # print index
        # print command
        command[limb_name][index] = f.data
    return call_back_func


if __name__ == '__main__':
    global command
    rospy.init_node('v9_subscriber')
    
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    rs.enable()
    
    left_limb = baxter_interface.limb.Limb("left")
    right_limb = baxter_interface.limb.Limb("right")

    arm_speed = 0.75
    #arm_speed = 1.0
    left_limb.set_joint_position_speed(arm_speed)
    right_limb.set_joint_position_speed(arm_speed)
    
    rospy.Subscriber("left_s0", Float64, call_back_gen('left', 0))
    rospy.Subscriber("left_s1", Float64, call_back_gen('left', 1))
    rospy.Subscriber("left_e0", Float64, call_back_gen('left', 2))
    rospy.Subscriber("left_e1", Float64, call_back_gen('left', 3))
    rospy.Subscriber("right_s0", Float64, call_back_gen('right', 0))
    rospy.Subscriber("right_s1", Float64, call_back_gen('right', 1))
    rospy.Subscriber("right_e0", Float64, call_back_gen('right', 2))
    rospy.Subscriber("right_e1", Float64, call_back_gen('right', 3))
    rospy.Subscriber("head_pan", Float64, call_back_gen('head', 0))
    rospy.Subscriber("head_nod", Float64, call_back_gen('head', 1))

    head_pan = rospy.Publisher('/robot/head/command_head_pan', HeadPanCommand, queue_size=1, tcp_nodelay=True)

    rospy.sleep(5)
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        angles = left_limb.joint_angles()
        angles['left_s0'] = command['left'][0]
        angles['left_s1'] = command['left'][1]
        angles['left_e0'] = command['left'][2]
        angles['left_e1'] = command['left'][3]
        angles['left_w0'] = 0
        angles['left_w1'] = 0
        angles['left_w2'] = 0
    
        # print angles
        left_limb.set_joint_positions(angles)
        
        angles = right_limb.joint_angles()
        angles['right_s0'] = command['right'][0]
        angles['right_s1'] = command['right'][1]
        angles['right_e0'] = command['right'][2]
        angles['right_e1'] = command['right'][3]
        angles['right_w0'] = 0
        angles['right_w1'] = 0
        angles['right_w2'] = 0
        right_limb.set_joint_positions(angles)

	head_pan.publish(float(command['head'][0]), 1, 0)
        
        rate.sleep()
