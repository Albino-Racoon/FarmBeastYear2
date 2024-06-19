#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import requests
import json
import os
from datetime import datetime

api_url_start = 'http://89.58.6.184:8000/fre2024/task3/start'
api_url_stop = 'http://89.58.6.184:8000/fre2024/task3/stop'
api_url_addposition = 'http://89.58.6.184:8000/fre2024/task3/add-position'

api_key = '5iFimPgBV8W0GhCAaRavqsid2iaq2WRh'

# Path to save the JSON file in the Downloads folder
download_folder = os.path.join(os.path.expanduser('~'), 'Downloads')
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
json_file_path = os.path.join(download_folder, f"position_data_{timestamp}.json")

# List to store the data
position_data_list = []

def send_start_signal():
    headers = {
        'Content-Type': 'application/json',
        'x-api-key': api_key
    }
    try:
        response = requests.post(api_url_start, headers=headers, json={})
        rospy.loginfo("Sent start signal")
        rospy.loginfo("Response Status: %s, Response Body: %s", response.status_code, response.text)
    except requests.exceptions.RequestException as e:
        rospy.logerr("Request failed: %s", e)

def send_stop_signal():
    headers = {
        'Content-Type': 'application/json',
        'x-api-key': api_key
    }
    try:
        response = requests.post(api_url_stop, headers=headers, json={})
        rospy.loginfo("Sent stop signal")
        rospy.loginfo("Response Status: %s, Response Body: %s", response.status_code, response.text)
    except requests.exceptions.RequestException as e:
        rospy.logerr("Request failed: %s", e)

def send_position_data(x, y):
    headers = {
        'Content-Type': 'application/json',
        'x-api-key': api_key
    }
    data = {
        "x": x,
        "y": y
    }
    try:
        response = requests.post(api_url_addposition, headers=headers, json=data)
        rospy.loginfo("Sent position data")
        rospy.loginfo("Response Status: %s, Response Body: %s", response.status_code, response.text)
    except requests.exceptions.RequestException as e:
        rospy.logerr("Request failed: %s", e)

def callback(data):
    x = data.linear.x  # Extract x coordinate
    y = data.linear.y  # Extract y coordinate

    position_data_list.append({"x": x, "y": y})
    send_position_data(x, y)

def stop_callback(data):
    if data.data == 'stop':
        rospy.loginfo("Received stop signal message")
        send_stop_signal()
        save_data_to_json()  # Save the data to JSON file
        rospy.signal_shutdown("Received stop signal")

def save_data_to_json():
    with open(json_file_path, 'w') as json_file:
        json.dump(position_data_list, json_file, indent=4)
    rospy.loginfo(f"Data saved to {json_file_path}")

def listener():
    rospy.Subscriber('position_data_topic', Twist, callback)
    rospy.Subscriber('stop_signal_topic', String, stop_callback)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('position_data_listener', anonymous=True)

    send_start_signal()
    listener()  # Start listening to the topic
