#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import requests
import json

###to pazi tu sicer nebi smelo bit problem ampak met jih kr tkle javno nezavarovano ni vedno najbol ok
api_url = 'http://89.58.6.184:8000/fre2024/task2/start'

api_key = '5iFimPgBV8W0GhCAaRavqsid2iaq2WRh'


def send_start_signal():
    headers = {
        'Content-Type': 'application/json',
        'x-api-key': api_key
    }
    try:

        response = requests.post(api_url, headers=headers, json={})
        rospy.loginfo("Sent start signal")
        rospy.loginfo("Response Status: %s, Response Body: %s", response.status_code, response.text)
    except requests.exceptions.RequestException as e:
        rospy.logerr("Request failed: %s", e)


def send_message(topic, msg, t):
    data = {
        'topic': topic,
        'message': {
            'linear': {
                'x': msg.linear.x,
                'y': msg.linear.y,
                'z': msg.linear.z
            },
            'angular': {
                'x': msg.angular.x,
                'y': msg.angular.y,
                'z': msg.angular.z
            }
        },
        'time': t.to_sec()
    }
    json_data = json.dumps(data)
    headers = {
        'Content-Type': 'application/json',
        'x-api-key': api_key
    }
    try:

        response = requests.post(api_url, headers=headers, json=data)
        rospy.loginfo("Sent message: %s", data)
        rospy.loginfo("Response Status: %s, Response Body: %s", response.status_code, response.text)
    except requests.exceptions.RequestException as e:
        rospy.logerr("Request failed: %s", e)


def message_callback(msg):
    current_time = rospy.Time.now()
    send_message('/farmbeast/odom', msg, current_time)


if __name__ == '__main__':
    rospy.init_node('continuous_topic_sender', anonymous=True)

    send_start_signal()

    rospy.Subscriber('/farmbeast/odom', Twist, message_callback)

    rospy.spin()
