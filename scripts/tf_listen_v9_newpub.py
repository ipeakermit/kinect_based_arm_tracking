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
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
from std_msgs.msg import Float64
#import turtlesim.srv
import math
import baxter_interface
from baxter_interface import CHECK_VERSION
from tf_user_tracker import UserTracker

import operator

mirror = False
times = 20
data_set = {'left':[[0]*times for i in range(4)], 'right':[[0]*times for i in range(4)]}

#new for v8
pub_list = {}
pub_list_raw = {}

user_id = "None"

def vector_length(v):
    return math.sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2])

def get_unit_vector(v):
    length = vector_length(v)
    return (v[0]/length, v[1]/length, v[2]/length)

def angle_query(v1, v2):
    p = v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2]
    m1 = vector_length(v1)
    m2 = vector_length(v2)
    cos = p/(m1*m2)
    return math.acos(cos)


def vector_cross_product(a, b):
    r1 = a[1]*b[2]-b[1]*a[2]
    r2 = a[2]*b[0]-b[2]*a[0]
    r3 = a[0]*b[1]-b[0]*a[1]
    return (r1, r2, r3)

def lookupTransform(listener,f1,f2,t):
    #print 'lookupTransform '+str(f1)+' '+str(f2)+' '+str(t)
    (xyz,ori), = listener.lookupTransform(f1, f2, t)
    print 'lookupTransform result '+str(xyz)
    #if xyz[2]<0.5:
    #  xyz[2]=0.5
    return result

def tf_2_angles(shoulder, shoulder_x, elbow, hand,  torso, head, base, listener, limb_name):
    try:
        (b_2_s, trash) = lookupTransform(listener, base, shoulder, rospy.Time(0))
        (b_2_s_x, trash) = lookupTransform(listener, base, shoulder_x, rospy.Time(0))
        (b_2_e, trash) = lookupTransform(listener, base, elbow, rospy.Time(0))
        (b_2_hand, trash) = lookupTransform(listener, base, hand, rospy.Time(0))
        (b_2_t, trash) = lookupTransform(listener, base, torso, rospy.Time(0))
        (b_2_head, trash) = lookupTransform(listener, base, head, rospy.Time(0))
        
        
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        print "tf exception "
        print "tf exception "+str(e)
        return {}

    b_2_s = list(b_2_s)
    b_2_s_x = list(b_2_s_x)
    b_2_e = list(b_2_e)
    b_2_hand = list(b_2_hand)
    b_2_t = list(b_2_t)
    b_2_head = list(b_2_head)
    
    s_2_s_x = map(operator.sub, b_2_s_x, b_2_s)
    s_2_t = map(operator.sub, b_2_t, b_2_s)
    s_2_e = map(operator.sub, b_2_e, b_2_s)
    e_2_hand = map(operator.sub, b_2_hand, b_2_e)
    
    head_2_t = map(operator.sub, b_2_t, b_2_head)
    
    my_v2 = head_2_t
    my_v3 = vector_cross_product(s_2_e, my_v2)
    
    my_theta_1 = angle_query(my_v3, s_2_s_x)
    
    if limb_name == 'right':
        radius_s0 = math.pi/4-my_theta_1
    else:
        radius_s0 = math.pi*3/4-my_theta_1
            
    if mirror:
        if limb_name == 'left': 
            radius_s0 = -radius_s0+math.pi/2
        else:
            radius_s0 = -radius_s0-math.pi/2

    my_theta_2 = angle_query(my_v2, s_2_e)
    radius_s1 = math.pi/2-my_theta_2
    
    my_v4 = vector_cross_product(s_2_e, e_2_hand)
    
    my_theta_3 = angle_query(my_v3,my_v4)
    
    
    radius_e0 = my_theta_3
    if limb_name == 'left':
        radius_e0 = -radius_e0
    
    my_theta_4 = angle_query(s_2_e,e_2_hand)
    radius_e1 = my_theta_4
    
    
    return {"s0":radius_s0, "s1":radius_s1, "e0":radius_e0, "e1":radius_e1}

def track_head(head, neck, base, listener):
    (b2n, trash) = lookupTransform(listener, base, neck, rospy.Time(0))
    (b2h, trash) = lookupTransform(listener, base, head, rospy.Time(0))
    dx = b2h[0] - b2n[0]
    dy = b2h[1] - b2n[1]
    dz = b2h[2] - b2n[2]
    print 'track_head '+str(dx)+' '+str(dy)+' '+str(dz)+' '+str(b2n)+' '+str(b2h)
    global pub_head
    pub_head.publish(dx*100.0)
    
def track_one_arm(shoulder, shoulder_x, elbow, hand, torso, head, base, listener, limb_name):
    print 'track_one_arm '+limb_name
    global pub_list_raw, pub_list
    d = tf_2_angles(shoulder, shoulder_x, elbow, hand, torso, head, base, listener, limb_name)
    if d == {}:
        print "tf listen failed."
        return
        
    radius_s0_raw = d["s0"]
    radius_s1_raw = d["s1"]
    radius_e0_raw = d["e0"]
    radius_e1_raw = d["e1"]

    print limb_name
    
    print
    for i in data_set[limb_name]:
        i.pop(0)

    data_set[limb_name][0].append(radius_s0_raw)
    data_set[limb_name][1].append(radius_s1_raw)
    data_set[limb_name][2].append(radius_e0_raw)
    data_set[limb_name][3].append(radius_e1_raw)

    command = [0.0, 0.0, 0.0, 0.0]
    for i in range(0, 4):
        sum = 0
        for j in range(0, times):
            sum = sum+data_set[limb_name][i][j]
        command[i] = sum/times
    
    radius_s0 = command[0]
    radius_s1 = command[1]
    radius_e0 = command[2]
    radius_e1 = command[3]
        
    print radius_s0, radius_s1, radius_e0, radius_e1

    print "haha"
    pub_list[limb_name+'_s0'].publish(radius_s0)
    print "hehe"

    pub_list[limb_name+'_s1'].publish(radius_s1)
    pub_list[limb_name+'_e0'].publish(radius_e0)
    pub_list[limb_name+'_e1'].publish(radius_e1)

def user_track_callback(msg):
    global user_id
    user_id = msg.data
    print("setting user_id to " + msg.data)

def talker():
    global pub_list_raw, pub_list
    rospy.init_node('v9_xnode', anonymous=True)
    
    #pub part
    pub_left_s0 = rospy.Publisher('left_s0', Float64, queue_size=10)
    pub_left_s1 = rospy.Publisher('left_s1', Float64, queue_size=10)
    pub_left_e0 = rospy.Publisher('left_e0', Float64, queue_size=10)
    pub_left_e1 = rospy.Publisher('left_e1', Float64, queue_size=10)

    pub_right_s0 = rospy.Publisher('right_s0', Float64, queue_size=10)
    pub_right_s1 = rospy.Publisher('right_s1', Float64, queue_size=10)
    pub_right_e0 = rospy.Publisher('right_e0', Float64, queue_size=10)
    pub_right_e1 = rospy.Publisher('right_e1', Float64, queue_size=10)
    pub_list = {'left_s0':pub_left_s0,
                'left_s1':pub_left_s1,
                'left_e0':pub_left_e0,
                'left_e1':pub_left_e1,
                'right_s0':pub_right_s0,
                'right_s1':pub_right_s1,
                'right_e0':pub_right_e0,
                'right_e1':pub_right_e1}

    #tf part
    listener = tf.TransformListener()

    global pub_head
    pub_head = rospy.Publisher('head_pan', Float64, queue_size=10)

    # Changed frame id's to match cob_body_tracker tf data
    # Need to add a function to choose a user from the skeletons being tracked
    # i.e. it will not always be user_7!

    user_tracker = rospy.Subscriber("user_tracking",
                                    String,
                                    user_track_callback)
    
    cob_prefix = '/cob_body_tracker_h9/'
    
    for_my_left_shoulder = '/right_shoulder'
    for_my_left_shoulder_x = '/left_shoulder'
    for_my_left_elbow = '/right_elbow'
    for_my_left_hand = '/right_hand'
    
    for_my_right_shoulder = '/left_shoulder'
    for_my_right_shoulder_x = '/right_shoulder'
    for_my_right_elbow = '/left_elbow'
    for_my_right_hand = '/left_hand'
    
    head = '/head'
    neck = '/neck'
    torso = '/torso'
    base = 'h9camera_depth_optical_frame'
    
    rospy.sleep(5)

    rate = rospy.Rate(10) # 10hz

    limb_1_name = 'left'
    limb_2_name = 'right'
    if mirror == True:
        limb_1_name = 'right'
        limb_2_name = 'left'
    
    while not rospy.is_shutdown():
        if user_id != "None":
            prefix = cob_prefix + user_id
            track_one_arm(prefix + for_my_left_shoulder
                         ,prefix + for_my_left_shoulder_x
                         ,prefix + for_my_left_elbow
                         ,prefix + for_my_left_hand
                         ,prefix + torso
                         ,prefix + head
                         ,base
                         ,listener
                         ,limb_1_name)
            track_one_arm(prefix + for_my_right_shoulder
                         ,prefix + for_my_right_shoulder_x
                         ,prefix + for_my_right_elbow
                         ,prefix + for_my_right_hand
                         ,prefix + torso
                         ,prefix + head
                         ,base
                         ,listener
                         ,limb_2_name)
            track_head(prefix + head, prefix + neck, base, listener)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
