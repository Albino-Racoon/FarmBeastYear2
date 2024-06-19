#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
import threading

def callback(data):
    if data.data:
        rospy.loginfo("Received True, activating spray_right and activating spray_left")
        spray_left_pub.publish(Bool(True))
        spray_right_pub.publish(Bool(True))
        threading.Timer(5.0, deactivate_spray).start()
    else:
        rospy.loginfo("Received False, deactivating spray_left and deactivating spray_right")
        spray_right_pub.publish(Bool(False))
        spray_left_pub.publish(Bool(False))
        threading.Timer(5.0, deactivate_spray).start()

def deactivate_spray():
    rospy.loginfo("Deactivating both sprays")
    spray_left_pub.publish(Bool(False))
    spray_right_pub.publish(Bool(False))

def listener():
    rospy.init_node('spray_controller', anonymous=True)

    rospy.Subscriber("/monitor_topic", Bool, callback)

    global spray_left_pub, spray_right_pub
    spray_left_pub = rospy.Publisher("/spray_left", Bool, queue_size=10)
    spray_right_pub = rospy.Publisher("/spray_right", Bool, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
