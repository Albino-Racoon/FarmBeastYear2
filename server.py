#!/usr/bin/env python
import rospy
import rosbag
import requests
import time
import json

# URL REST API-ja za začetek naloge 2
api_url = 'http://89.58.6.184:8000/fre2024/task2/start'
# API ključ za avtentifikacijo
api_key = '5iFimPgBV8W0GhCAaRavqsid2iaq2WRh'


def send_start_signal():
    """ Pošlje signal za začetek naloge preko HTTP POST. """
    headers = {
        'Content-Type': 'application/json',
        'x-api-key': api_key
    }
    try:
        # Pošlji signal za začetek naloge preko HTTP POST
        response = requests.post(api_url, headers=headers, json={})
        rospy.loginfo("Sent start signal")
        rospy.loginfo("Response Status: %s, Response Body: %s", response.status_code, response.text)
    except requests.exceptions.RequestException as e:
        rospy.logerr("Request failed: %s", e)


def continuous_sender(bag_path):
    """ Neprekinjeno pošilja sporočila iz ROS bag datoteke in pošlje signal za začetek naloge. """
    rospy.init_node('continuous_bag_sender', anonymous=True)

    # Pošlji signal za začetek naloge
    send_start_signal()

    with rosbag.Bag(bag_path, 'r') as bag:
        while not rospy.is_shutdown():
            for topic, msg, t in bag.read_messages():
                # Prilagodite in pošljite sporočilo glede na vašo potrebo
                send_message(topic, msg, t)
                time.sleep(0.1)  # Časovna zakasnitev za preprečevanje preobremenitve strežnika
                if rospy.is_shutdown():
                    break


def send_message(topic, msg, t):
    """ Pošlje posamezno sporočilo preko HTTP POST. """
    # Pripravi podatke za pošiljanje
    data = {
        'topic': topic,
        'message': str(msg),
        'time': t.to_sec()
    }
    json_data = json.dumps(data)  # Pretvori podatke v JSON format
    headers = {
        'Content-Type': 'application/json',
        'x-api-key': api_key
    }
    try:
        # Pošlji podatke preko HTTP POST
        response = requests.post(api_url, headers=headers, json=data)
        rospy.loginfo("Sent message: %s", data)
        rospy.loginfo("Response Status: %s, Response Body: %s", response.status_code, response.text)
    except requests.exceptions.RequestException as e:
        rospy.logerr("Request failed: %s", e)


if __name__ == '__main__':
    bag_path = "/home/rakun/Downloads/4m_ravno_obrat_4m_ravno.bag"  # Pot do vaše bag datoteke
    continuous_sender(bag_path)
