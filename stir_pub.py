#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point

# List of coordinates
coordinates = [
    {"x": 1.34, "y": 1.8},
    {"x": 0.28, "y": 6.88},
    {"x": 0.0, "y": 1.02},
    {"x": -2.06, "y": 2.61},
    {"x": -0.38, "y": 3.24},
    {"x": 2.37, "y": 3.88},
    {"x": -3.0, "y": 4.51},
    {"x": -1.9, "y": 6.17},
    {"x": 2.31, "y": 5.45},
    {"x": -0.89, "y": 5.97}
]

def publish_coordinates():
    # Initialize the ROS node
    rospy.init_node('coordinate_publisher', anonymous=True)
    # Create a publisher that will publish to the /coordinates topic
    pub = rospy.Publisher('/coordinates', Point, queue_size=10)
    # Set the loop rate
    rate = rospy.Rate(1)  # Approximately one message every 30 seconds= 0.333 spremeni pred zagonom na robotu !!!!!!!!!!!!!!!!!!!!

    index = 0  # Start index

    while not rospy.is_shutdown():
        # Get the current coordinate
        coord = coordinates[index]
        # Create a Point message
        point = Point()
        point.x = coord["x"]
        point.y = coord["y"]
        point.z = 0.0  # Assuming z is 0.0 as it is not specified

        # Log the coordinate being published
        rospy.loginfo(f"Publishing coordinates: ({point.x}, {point.y})")

        # Publish the coordinate
        pub.publish(point)

        # Increment the index and loop back to the start if needed
        index = (index + 1) % len(coordinates)

        # Sleep for 30 seconds
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_coordinates()
    except rospy.ROSInterruptException:
        pass
