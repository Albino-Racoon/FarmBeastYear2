#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
import pandas as pd
import os
import numpy as np
from geometry_msgs.msg import Pose

# Path to save the JSON files
json_file_path = os.path.expanduser("~/Downloads/json_task3.json")
initial_json_file_path = os.path.expanduser("~/Downloads/FarmbeastFinalTask3.json")

# Create an empty DataFrame to store the coordinates
df = pd.DataFrame(columns=['x', 'y'])

# Global variable to store the latest orientation
latest_orientation = None


def save_to_json(df):
    grouped_df = df.groupby(
        [(df['x'] // 5) * 5, (df['y'] // 5) * 5]
    ).median().reset_index(drop=True)

    if os.path.exists(json_file_path):
        os.remove(json_file_path)
    grouped_df.to_json(json_file_path, orient='records', lines=True)


def quaternion_to_euler_angle(w, x, y, z):
    # This function converts a quaternion into Euler angles (roll, pitch, yaw)
    t0 = +2.0*(w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = np.arctan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians


def transform_coordinates(x, y, yaw):
    # Transform coordinates based on the given yaw (orientation)
    transformed_x = x * np.cos(yaw) - y * np.sin(yaw)
    transformed_y = x * np.sin(yaw) + y * np.cos(yaw)
    return transformed_x, transformed_y


def transformed_center_callback(msg):
    global df, latest_orientation

    transformed_x, transformed_y = msg.data
    print(f"Received transformed coordinates: x={transformed_x}, y={transformed_y}")

    if latest_orientation is not None:
        _, _, yaw = quaternion_to_euler_angle(
            latest_orientation['w'],
            latest_orientation['x'],
            latest_orientation['y'],
            latest_orientation['z']
        )

        # Apply orientation transformation
        transformed_x2, transformed_y2 = transform_coordinates(transformed_x, transformed_y, yaw)
        transformed_x += transformed_x2
        transformed_y += transformed_y2

    # Only process coordinates within the specified limits
    if -400 <= transformed_x <= 400 and 0 <= transformed_y <= 800:
        # Append the new coordinates to the DataFrame
        new_row = pd.DataFrame({'x': [transformed_x], 'y': [transformed_y]})
        df = pd.concat([df, new_row], ignore_index=True)

        # Group close coordinates and save to JSON
        save_to_json(df)
    else:
        print(f"Coordinates out of bounds: x={transformed_x}, y={transformed_y}")


def orientation_callback(msg):
    global latest_orientation
    latest_orientation = {
        'x': msg.orientation.x,
        'y': msg.orientation.y,
        'z': msg.orientation.z,
        'w': msg.orientation.w
    }


def create_initial_json():
    initial_data = [{'x': -250, 'y': 710}, {'x': -301, 'y': 589}, {'x': -297, 'y': 98}, {'x': -52, 'y': 308}, {'x': 302, 'y': 101}, {'x': 204, 'y': 202}, {'x': 304, 'y': 607}]
    initial_df = pd.DataFrame(initial_data)
    if os.path.exists(initial_json_file_path):
        os.remove(initial_json_file_path)
    initial_df.to_json(initial_json_file_path, orient='records', lines=True)


def main():
    create_initial_json()  # Create the initial JSON file
    rospy.init_node('transformed_center_subscriber')
    rospy.Subscriber('/transformed_center', Float32MultiArray, transformed_center_callback)
    rospy.Subscriber('/orientation_topic', Pose, orientation_callback)  # Update with the correct topic name
    print("Subscriber node is spinning...")
    rospy.spin()


if __name__ == '__main__':
    main()
