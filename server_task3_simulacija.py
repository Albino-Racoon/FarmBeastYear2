#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import random

def publisher():
    pub = rospy.Publisher('position_data_topic', Twist, queue_size=10)
    stop_pub = rospy.Publisher('stop_signal_topic', String, queue_size=10)
    rospy.init_node('position_data_publisher', anonymous=True)
    rate = rospy.Rate(1)  # 1 Hz

    for i in range(10):  # Publish 10 messages
        x = random.randint(0, 100)  # Random x coordinate
        y = random.randint(0, 100)  # Random y coordinate

        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y

        rospy.loginfo(f"Publishing X: {x}, Y: {y}")
        pub.publish(twist)
        rate.sleep()

    # Publish stop message
    stop_msg = String()
    stop_msg.data = 'stop'
    stop_pub.publish(stop_msg)
    rospy.loginfo("Published stop message")

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
