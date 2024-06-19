#!/usr/bin/env python

import rospy
import math
import tf
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geographic_msgs.msg import GeoPointStamped
from tf.transformations import euler_from_quaternion

class PointToPointDriver:
    def __init__(self):
        rospy.init_node('point_to_point_driver')

        # Initialize publishers and subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pose_sub = rospy.Subscriber('/filter/positionlla', GeoPointStamped, self.pose_callback)
        self.orientation_sub = rospy.Subscriber('/filter/quaternion', Imu, self.orientation_callback)

        self.current_position = None
        self.current_orientation = None

    def pose_callback(self, msg):
        self.current_position = msg.position

    def orientation_callback(self, msg):
        self.current_orientation = msg.orientation

    def get_current_pose(self):
        if self.current_position and self.current_orientation:
            position = self.current_position
            orientation_q = [self.current_orientation.x, self.current_orientation.y, self.current_orientation.z, self.current_orientation.w]
            return position, orientation_q
        else:
            return None, None

    def rotate_to_angle(self, target_angle):
        position, orientation_q = self.get_current_pose()
        if orientation_q:
            (roll, pitch, yaw) = euler_from_quaternion(orientation_q)
            angle_diff = target_angle - yaw

            twist = Twist()
            twist.angular.z = angle_diff * 0.5  # Proportional control for simplicity

            while abs(angle_diff) > 0.01:  # Continue rotating until the angle difference is small
                self.cmd_vel_pub.publish(twist)
                rospy.sleep(0.1)
                _, orientation_q = self.get_current_pose()
                (roll, pitch, yaw) = euler_from_quaternion(orientation_q)
                angle_diff = target_angle - yaw

            twist.angular.z = 0
            self.cmd_vel_pub.publish(twist)

    def drive_to_point(self, target_lat, target_lon):
        current_position, orientation_q = self.get_current_pose()
        if current_position:
            current_lat = current_position.latitude
            current_lon = current_position.longitude

            distance = self.haversine_distance(current_lat, current_lon, target_lat, target_lon)
            target_angle = math.atan2(target_lat - current_lat, target_lon - current_lon)

            self.rotate_to_angle(target_angle)

            twist = Twist()
            twist.linear.x = distance * 0.5  # Proportional control for simplicity

            while distance > 0.1:  # Continue driving until the distance is small
                self.cmd_vel_pub.publish(twist)
                rospy.sleep(0.1)
                current_position, _ = self.get_current_pose()
                current_lat = current_position.latitude
                current_lon = current_position.longitude
                distance = self.haversine_distance(current_lat, current_lon, target_lat, target_lon)

            twist.linear.x = 0
            self.cmd_vel_pub.publish(twist)

    def haversine_distance(self, lat1, lon1, lat2, lon2):
        R = 6371.0  # Earth radius in kilometers
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = math.sin(dlat / 2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c
        return distance

    def run(self):
        # Define your target points (latitude, longitude)
        target_points = [(47.397742, 8.545593), (47.398742, 8.546593), (47.399742, 8.547593)]  # Example points

        for point in target_points:
            target_lat, target_lon = point
            self.drive_to_point(target_lat, target_lon)

if __name__ == '__main__':
    driver = PointToPointDriver()
    driver.run()
