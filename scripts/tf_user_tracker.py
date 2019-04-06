#!/usr/bin/env python
import rospy
import tf
from tf2_msgs.msg import TFMessage
import time

class UserTracker:
    def __init__(self):
        rospy.init_node('user_tracker')
        self.tflistener = tf.TransformListener()
        self.sub = rospy.Subscriber("/tf", TFMessage, self.callback)
        self.user_str = None
        self.time_at_user_present = 0.0

    def callback(self, msg):
        user = None
        for item in msg.transforms:
            if (item.child_frame_id.find("/cob_body_tracker/") != -1 and
                item.child_frame_id.find("/head") != -1):
                user = item.child_frame_id.split("/")[2]

        if time.time() - self.time_at_user_present > 3:
            self.user_str = None

        if user != None:
            if self.user_str == None:
                print("tracking " + user)
                self.user_str = user
                self.time_at_user_present = time.time()
            elif user == self.user_str:
                self.time_at_user_present = time.time()

    def get_user(self):
        return self.user_str
    
    def run(self):
        rospy.spin()
        
if __name__ == "__main__":
    ut = UserTracker()
    ut.run()
