#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
import json
import os

# Pot do JSON ppol spremenit na kljucek
json_file_path = os.path.expanduser('~/Downloads/data.json')


initial_data = {
    "data": []
}
with open(json_file_path, 'w') as f:
    json.dump(initial_data, f)

def message_callback(msg):
    """ Callback funkcija za obdelavo prejetih sporočil iz ROS topic-a. """
    global json_file_path


    with open(json_file_path, 'r') as f:
        data = json.load(f)


    row_number = len(data["data"]) + 1
    new_entry = {
        "row": row_number,
        "number": msg.data
    }
    data["data"].append(new_entry)


    with open(json_file_path, 'w') as f:
        json.dump(data, f, indent=4)

    rospy.loginfo("Added data: %s", new_entry)

if __name__ == '__main__':
    rospy.init_node('json_writer', anonymous=True)
    rospy.Subscriber('/numbers', Int32, message_callback)
    rospy.spin()
