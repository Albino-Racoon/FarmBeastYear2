#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
import cv2
import numpy as np
from pathlib import Path
import sys

# Ensure YOLOv8 is in the system path
sys.path.insert(0, str(Path(__file__).resolve().parent / 'yolov8'))

# Import YOLOv8
from ultralytics import YOLO

# Initialize YOLO model
model = YOLO('home/rakun/Downloads/best.pt')  # Update this to the path of your YOLOv8 model

bridge = CvBridge()

def image_callback(msg):
    # Convert ROS Image message to OpenCV image
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    # Prepare image for YOLO
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = model(img)

    # Draw detections and publish to RViz
    for result in results:
        for box in result.boxes:
            xyxy = box.xyxy[0].numpy()
            conf = box.conf[0].numpy()
            cls = box.cls[0].numpy()
            label = f'{model.names[int(cls)]} {conf:.2f}'
            plot_one_box(xyxy, frame, label=label, color=(0, 255, 0))

    # Convert OpenCV image back to ROS Image message
    result_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
    result_pub.publish(result_msg)

def plot_one_box(xyxy, img, color=(0, 255, 0), label=None, line_thickness=3):
    c1, c2 = (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3]))
    cv2.rectangle(img, c1, c2, color, thickness=line_thickness, lineType=cv2.LINE_AA)
    if label:
        font_thickness = max(line_thickness - 1, 1)
        t_size = cv2.getTextSize(label, 0, fontScale=line_thickness / 3, thickness=font_thickness)[0]
        c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
        cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled
        cv2.putText(img, label, (c1[0], c1[1] - 2), 0, line_thickness / 3, [225, 255, 255],
                    thickness=font_thickness, lineType=cv2.LINE_AA)

if __name__ == '__main__':
    rospy.init_node('yolo_node')
    result_pub = rospy.Publisher('/yolo/results', Image, queue_size=1)
    image_sub = rospy.Subscriber('/camera/image_raw', Image, image_callback)

    rospy.spin()
