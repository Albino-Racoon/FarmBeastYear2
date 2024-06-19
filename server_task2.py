import rospy
from geometry_msgs.msg import Twist
import requests
import json
import os
from datetime import datetime

api_url_start = 'http://89.58.6.184:8000/fre2024/task2/start'
api_url_stop = 'http://89.58.6.184:8000/fre2024/task2/stop'
api_url_addrow = 'http://89.58.6.184:8000/fre2024/task2/add-row'
api_key = '5iFimPgBV8W0GhCAaRavqsid2iaq2WRh'

# Path to save the JSON file in the Downloads folder
download_folder = os.path.join(os.path.expanduser('~'), 'Downloads')
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
json_file_path = os.path.join(download_folder, f"row_data_{timestamp}.json")

# List to store the data
row_data_list = []

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

def send_row_data(row_number, plant_count):
    headers = {
        'Content-Type': 'application/json',
        'x-api-key': api_key
    }
    data = {
        "plant_count": plant_count,
        "row_number": row_number
    }
    try:
        response = requests.post(api_url_addrow, headers=headers, json=data)
        rospy.loginfo("Sent row data")
        rospy.loginfo("Response Status: %s, Response Body: %s", response.status_code, response.text)
    except requests.exceptions.RequestException as e:
        rospy.logerr("Request failed: %s", e)

def callback(data):
    plant_count = data.linear.x  # Extract plant count
    row_number = data.linear.y  # Extract row number

    if plant_count == -1 and row_number == -1:
        rospy.loginfo("Received stop signal message")
        send_stop_signal()
        save_data_to_json()  # Save the data to JSON file
        rospy.signal_shutdown("Received stop signal")
    else:
        row_data_list.append({"row_number": row_number, "plant_count": plant_count})
        send_row_data(row_number, plant_count)

def save_data_to_json():
    with open(json_file_path, 'w') as json_file:
        json.dump(row_data_list, json_file, indent=4)
    rospy.loginfo(f"Data saved to {json_file_path}")

def listener():
    rospy.Subscriber('row_data_topic', Twist, callback)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('continuous_topic_sender', anonymous=True)

    send_start_signal()
    listener()  # Start listening to the topic
