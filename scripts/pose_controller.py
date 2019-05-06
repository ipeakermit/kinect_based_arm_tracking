#!/usr/bin/env python
import roslib
roslib.load_manifest('kinect_based_arm_tracking')
import rospy
import math
import tf
import geometry_msgs.msg
import math
import baxter_interface
from std_msgs.msg import Float64
from baxter_interface import CHECK_VERSION
from kinect_based_arm_tracking.msg import baxterpose

import operator

# shoulder_a is the shoulder of the intested arm

'''
    This script sets a subscriber for each joint and when a subscriber receives a message it sets a corresponding
    element in the "command" dictionary, the main function also updates baxters arm joints with data from the
    command dictionary to corresponding joints every 1/10th of a second
'''

command = {"left": [0.0]*4, "right": [0.0]*4}

def pose_callback(data):
    left_limb = baxter_interface.limb.Limb("left")
    right_limb = baxter_interface.limb.Limb("right")
    
    angles = left_limb.joint_angles()
    angles['left_s0'] = data.left_s0
    angles['left_s1'] = data.left_s1
    angles['left_e0'] = data.left_e0
    angles['left_e1'] = data.left_e1
    angles['left_w0'] = 0
    angles['left_w1'] = 0
    angles['left_w2'] = 0
    
    # print angles
    left_limb.set_joint_positions(angles)
        
    angles = right_limb.joint_angles()
    angles['right_s0'] = data.right_s0
    angles['right_s1'] = data.right_s1
    angles['right_e0'] = data.right_e0
    angles['right_e1'] = data.right_e1
    angles['right_w0'] = 0
    angles['right_w1'] = 0
    angles['right_w2'] = 0
    right_limb.set_joint_positions(angles)

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
    rospy.init_node('v8_subscriber')
    
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    rs.enable()
    
    left_limb = baxter_interface.limb.Limb("left")
    right_limb = baxter_interface.limb.Limb("right")

    rospy.Subscriber("bax_pose", baxterpose, pose_callback)

    rospy.spin()
    
    #rospy.Subscriber("left_s0", Float64, call_back_gen('left', 0))
    #rospy.Subscriber("left_s1", Float64, call_back_gen('left', 1))
    #rospy.Subscriber("left_e0", Float64, call_back_gen('left', 2))
    #rospy.Subscriber("left_e1", Float64, call_back_gen('left', 3))
    #rospy.Subscriber("right_s0", Float64, call_back_gen('right', 0))
    #rospy.Subscriber("right_s1", Float64, call_back_gen('right', 1))
    #rospy.Subscriber("right_e0", Float64, call_back_gen('right', 2))
    #rospy.Subscriber("right_e1", Float64, call_back_gen('right', 3))

    #rospy.sleep(5)
    #rate = rospy.Rate(10.0)

    #while not rospy.is_shutdown():
    #    angles = left_limb.joint_angles()
    #    angles['left_s0'] = command['left'][0]
    #    angles['left_s1'] = command['left'][1]
    #    angles['left_e0'] = command['left'][2]
    #    angles['left_e1'] = command['left'][3]
    #    angles['left_w0'] = 0
    #    angles['left_w1'] = 0
    #    angles['left_w2'] = 0
    
        # print angles
    #    left_limb.set_joint_positions(angles)
        
    #    angles = right_limb.joint_angles()
    #    angles['right_s0'] = command['right'][0]
    #    angles['right_s1'] = command['right'][1]
    #    angles['right_e0'] = command['right'][2]
    #    angles['right_e1'] = command['right'][3]
    #    angles['right_w0'] = 0
    #    angles['right_w1'] = 0
    #    angles['right_w2'] = 0
    #    right_limb.set_joint_positions(angles)
        
    #    rate.sleep()
