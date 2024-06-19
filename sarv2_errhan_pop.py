import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import requests
import json
import os
from datetime import datetime

# API URLs and key
api_url_start = 'http://89.58.6.184:8000/fre2024/task2/start'
api_url_stop = 'http://89.58.6.184:8000/fre2024/task2/stop'
api_url_addrow = 'http://89.58.6.184:8000/fre2024/task2/add-row'
api_key = '5iFimPgBV8W0GhCAaRavqsid2iaq2WRh'

# Path to save the JSON file in the Downloads folder
download_folder = os.path.join(os.path.expanduser('~'), 'Downloads')
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
json_file_path = os.path.join(download_folder, f"row_data_{timestamp}.json")
json_backup_path = os.path.join(download_folder, "json_backup.json")

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

def callback_terminate(data):
    if data.buttons[0] == 1:
        rospy.loginfo("Received stop signal message")
        send_stop_signal()
        save_data_to_json()  # Save the data to JSON file
        rospy.signal_shutdown("Received stop signal")

def callback(data):
    plant_count = data.linear.x  # Extract plant count
    row_number = data.linear.y  # Extract row number

    # Filter out invalid data
    if row_number == -1.0 and plant_count == -1.0:
        return

    row_data_list.append({"row_number": row_number, "plant_count": plant_count})
    try:
        send_row_data(row_number, plant_count)
    except requests.exceptions.RequestException:
        rospy.logerr("Failed to send row data, saving to local backup")

    save_data_to_backup()

def save_data_to_json():
    with open(json_file_path, 'w') as json_file:
        json.dump(row_data_list, json_file, indent=4)
    rospy.loginfo(f"Data saved to {json_file_path}")

def save_data_to_backup():
    with open(json_backup_path, 'w') as json_file:
        json.dump(row_data_list, json_file, indent=4)
    rospy.loginfo(f"Backup data saved to {json_backup_path}")

def listener():
    rospy.Subscriber('stevilo_koruze_konec_vrste', Twist, callback)
    rospy.Subscriber('joy', Joy, callback_terminate)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('continuous_topic_sender', anonymous=True)

    # Delete json_backup file if it exists
    if os.path.exists(json_backup_path):
        os.remove(json_backup_path)
        rospy.loginfo(f"Deleted existing backup file {json_backup_path}")

    send_start_signal()
    listener()  # Start listening to the topic in the beginning
