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
    '''
        Default Constructor
    '''
    def __init__(self):
        rospy.init_node('user_tracker')
        self.sub = rospy.Subscriber("/tf", TFMessage, self.callback)
        self.pub = rospy.Publisher("user_tracking", String, queue_size=10)
        self.user_str = None
        self.time_at_user_present = None

    '''
        Callback, runs on receipt msg on topic /tf
    '''
    def callback(self, msg):
        user = None
        # Find a user id from cob_body_tracker child frames within /tf
        for item in msg.transforms:
            if (item.child_frame_id.find("/cob_body_tracker/") != -1 and
                item.child_frame_id.find("/head") != -1):
                '''
                    Extract user id from child frame id in format?: 
                    /cob_body_tracker/{USER_ID}/head
                    TODO: It's definitely index 2, the format must be 
                    slightly different and i cant remember it off the 
                    top of my head
                    
                '''
                user = item.child_frame_id.split("/")[2]

        '''
            If user being tracked has not been present for 10 seconds,
            reset to new user
        '''
        if (self.time_at_user_present != None and
            time.time() - self.time_at_user_present > 10):
            self.user_str = None
            self.pub.publish("None")
            self.time_at_user_present = None

        if user != None:
            '''
                If a new user is being tracked, publish the id and
                set internal variables. Also start a timer
            '''
            if self.user_str == None:
                print("tracking " + user)
                self.user_str = user
                self.time_at_user_present = time.time()
                self.pub.publish(user)
            '''
                If a user is already being tracked and they appear in
                the /tf data, reset the timer
            '''
            elif user == self.user_str:
                self.time_at_user_present = time.time()


if __name__ == "__main__":
    ut = UserTracker()
    rospy.spin()
