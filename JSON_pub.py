#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
import random
import time

def publish_random_numbers():
    pub = rospy.Publisher('/numbers', Int32, queue_size=10)
    rospy.init_node('random_number_publisher', anonymous=True)
    rate = rospy.Rate(1/3)  # 1 objava na 3 sekunde

    while not rospy.is_shutdown():
        random_number = random.randint(1, 100)
        rospy.loginfo("Publishing: %d", random_number)
        pub.publish(random_number)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_random_numbers()
    except rospy.ROSInterruptException:
        pass
