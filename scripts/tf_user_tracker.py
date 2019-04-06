#!/usr/bin/env python
import rospy
import tf
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
import time

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
            if (item.child_frame_id.find("/cob_body_tracker/") != -1 and
                item.child_frame_id.find("/head") != -1):
                user = item.child_frame_id.split("/")[2]

        if (self.time_at_user_present != None and
            time.time() - self.time_at_user_present > 10):
            self.user_str = None
            self.pub.publish("None")
            self.time_at_user_present = None

        if user != None:
            if self.user_str == None:
                print("tracking " + user)
                self.user_str = user
                self.time_at_user_present = time.time()
                self.pub.publish(user)
            elif user == self.user_str:
                self.time_at_user_present = time.time()


if __name__ == "__main__":
    ut = UserTracker()
    rospy.spin()
