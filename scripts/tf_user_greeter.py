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
   
   Author: Lachlan Clulow
   Last Modified: 3/6/19
"""

import rospy
import tf
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String, Bool
import time
#from geometry_msgs import TransformStamped
from baxter_core_msgs.msg import HeadPanCommand
import random
import math

global greetings
greetings = ['hello','hi','good morning','morning','wuminjeka --- welcome']

def greet():
   index = random.randint(0,len(greetings) - 1)
   t = time.localtime() # gives you an actual struct_time object
   h = t.tm_hour # gives you the hour part as an integer
   g = greetings[index]
   if g == 'good morning':
       if h>=12:
           g = 'good afternoon'
       if h>17:
           g = 'good evening'
   if g == 'morning':
       if h>=12:
           g = 'afternoon'
       if h>17:
           g = 'evening'
   return g

class TrackedUser:
    last_seen_at = 0
    tr = None
    greeted = False

    def __str__(self):
       return 'user: @ '+str(self.last_seen_at)+' '+str(self.tr)+'\n- greeted:'+str(self.greeted)

global users
users = {}

'''
    This class is used to filter /tf data for cob body tracker frames and 
    track a single user by their id, it published on the /user_tracking
    topic a user's id
'''
class UserGreeter:
    def __init__(self):
        rospy.init_node('user_greeter')
        self.sub = rospy.Subscriber("/tf", TFMessage, self.callback)
        self.headpan = rospy.Publisher("/robot/head/command_head_pan", HeadPanCommand, queue_size=5, tcp_nodelay=True)
        self.headnod = rospy.Publisher("/robot/head/command_head_nod", Bool, queue_size=5, tcp_nodelay=True)
        self.video = rospy.Publisher("/rosie/video_request", String, queue_size=5, tcp_nodelay=True)
        self.speech = rospy.Publisher("/red/text2speech", String, queue_size=5, tcp_nodelay=True)
        rospy.Timer(rospy.Duration(1.0/8), self.worker)
        self.user_str = None

    def worker(self, msg):
        # find front
        # timeout if gone
        global users
        closest = 10000;
        closest_user = None
        # timeout / find closest
        for user in users:
            tu = users[user]
            # WARN: possible race here, tu.tr is set in subscriber thread
            if tu.last_seen_at + 1 < time.time() and tu.tr != None:
                print 'Gone:',user
                tu.tr = None
            tu.last_seen_at = time.time()
            if tu.tr != None:
                if tu.tr.z < closest:
                    closest_user = user
                    closest = tu.tr.z
        if closest_user != None:
            tu = users[closest_user]
            print 'Closest:',closest_user,tu
            theta = math.atan2(tu.tr.x,tu.tr.z)
            print 'angle',theta
            self.headpan.publish(theta * 1.5, 1, 0)
            if not tu.greeted:
                self.headnod.publish(True)
                self.video.publish('3b')
                print 'Greet:',closest_user
                #self.speech.publish(greet())
            tu.greeted = True

    def callback(self, msg):
        user = None
        for item in msg.transforms:
            if (item.child_frame_id.find("/cob_body_tracker/") != -1 and
                item.child_frame_id.find("/head") != -1):
                user = item.child_frame_id.split("/")[2]
                if not user in users:
                    users[user] = TrackedUser()
                tu = users[user]
                tu.tr = item.transform.translation
                if tu.last_seen_at + 1 < time.time():
                    print 'Refresh',user
                    tu.last_seen_at = time.time()
                    tu.greeted = False

if __name__ == "__main__":
    ut = UserGreeter()
    rospy.spin()
