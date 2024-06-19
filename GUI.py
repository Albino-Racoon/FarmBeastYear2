#!/usr/bin/env python
import rospy
import rosbag_pandas
import pandas as pd
import requests
from std_msgs.msg import String


def load_data_from_bag(bag_path, topic):
    """ Naloži podatke iz ROS bag datoteke v DataFrame. """
    try:
        df = rosbag_pandas.bag_to_dataframe(bag_path, include=topic)
        rospy.loginfo("Data loaded successfully from bag file.")
        return df
    except Exception as e:
        rospy.logerr("Failed to load data from bag: %s", e)
        return pd.DataFrame()  # Vrni prazen DataFrame v primeru napake


def send_dataframe(df):
    """ Pretvori DataFrame v JSON in pošlji preko HTTP POST. """
    json_data = df.to_json()
    try:
        response = requests.post('http://localhost:5000/receive', json={'data': json_data})
        rospy.loginfo("Response Status: %s, Response Body: %s", response.status_code, response.text)
    except requests.exceptions.RequestException as e:
        rospy.logerr("Request failed: %s", e)


def sender(bag_file, topic):
    rospy.init_node('data_sender', anonymous=True)
    bag_path = f"/home/rakun/Downloads/{bag_file}"

    df = load_data_from_bag(bag_path, topic)
    if not df.empty:
        send_dataframe(df)
    else:
        rospy.logerr("Dataframe is empty, no data sent.")


if __name__ == '__main__':
    bag_files = ["4m_ravno_obrat_4m_ravno.bag", "odom_reset_pri_obracanju.bag"]
    topic = "/topic_to_send"  # Primer teme; zamenjajte z dejansko temo
    for bag_file in bag_files:
        sender(bag_file, topic)
