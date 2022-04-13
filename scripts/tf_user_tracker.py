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
from std_msgs.msg import String
import time

'''
    This class is used to filter /tf data for cob body tracker frames and 
    track a single user by their id, it published on the /user_tracking
    topic a user's id
'''
class UserTracker:
    def __init__(self):
        rospy.init_node('user_tracker')
        self.sub = rospy.Subscriber("/tf", TFMessage, self.callback)
        self.pub = rospy.Publisher("user_tracking", String, queue_size=10)
        self.user_str = None
        self.time_at_user_present = None

    def callback(self, msg):
        user = None
        for item in msg.transforms:
            if (item.child_frame_id.find("/cob_body_tracker") != -1 and
                item.child_frame_id.find("/head") != -1):
                user = item.child_frame_id.split("/")[2]
                #print("user = "+user)

        if (self.time_at_user_present != None and
            time.time() - self.time_at_user_present > 10):
              print("no user")
              self.user_str = None
              self.pub.publish("None")
              self.time_at_user_present = None

        if user != None:
            if self.user_str == None:
                print("saw user " + user)
                self.user_str = user
                self.time_at_user_present = time.time()
                self.pub.publish(user)
            elif user == self.user_str:
                self.time_at_user_present = time.time()


if __name__ == "__main__":
    ut = UserTracker()
    rospy.spin()
