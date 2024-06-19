#!/usr/bin/env python3
import rospy
import rosbag
from sensor_msgs.msg import Image
import os

def publish_bag():
    # Initialize ROS node
    rospy.init_node('bag_publisher', anonymous=True)
    
    # Create a publisher for the image topic
    image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    
    # Path to the ROS bag file
    bag_file = '/home/rakun/catkin_ws/src/farmbeast_simulation/src/FB_uporabno/2024-06-10-20-41-30.bag'
    
    # Open the bag file
    bag = rosbag.Bag(bag_file)
    
    # Set the loop rate
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Iterate through the messages in the bag
        for topic, msg, t in bag.read_messages(topics=['/camera/image_raw']):
            # Publish each message
            image_pub.publish(msg)
            # Sleep for a bit before publishing the next message
            rate.sleep()

    # Close the bag file
    bag.close()

if __name__ == '__main__':
    try:
        publish_bag()
    except rospy.ROSInterruptException:
        pass
