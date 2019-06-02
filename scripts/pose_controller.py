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


if __name__ == '__main__':
    rospy.init_node('pose_subscriber')
    
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    rs.enable()

    rospy.Subscriber("bax_pose", baxterpose, pose_callback)

    rospy.spin()
