#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
import pyrealsense2 as rs
import torch
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import json


class ShapeDetectionNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('shape_detection_node', anonymous=True)

        # Load the YOLO model
        self.model = YOLO("/home/rakun/Downloads/tekmovanje_best.pt")
        rospy.loginfo("YOLO model loaded successfully.")

        # Create a CvBridge object
        self.bridge = CvBridge()

        # Subscribe to image and depth topics
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        rospy.loginfo("Subscribed to /camera/color/image_raw topic.")

        # Publisher for detected shapes data
        self.shape_pub = rospy.Publisher('/detected_shapes', String, queue_size=10)

        # Initialize variables
        self.rgb_image = None

        rospy.spin()

    def image_callback(self, data):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            rospy.loginfo("Received an image.")
            self.process_image()
        except CvBridgeError as e:
            rospy.logerr(e)

    def process_image(self):
        if self.rgb_image is None:
            return

        # Perform detection
        results = self.model(self.rgb_image)
        detected_shapes = []

        image = self.rgb_image.copy()

        if results:
            rospy.loginfo(f"Objects detected: {len(results)}")
        else:
            rospy.loginfo("No objects detected.")

        for result in results:
            boxes = result.boxes.data.cpu().numpy()
            rospy.loginfo(f"Detected boxes: {boxes}")
            for box in boxes:
                middle = [box[0] + (box[2] - box[0]) / 2, box[1] + (box[3] - box[1]) / 2]
                pose = [middle[0], middle[1], 0]  # Only x and y positions available in 2D image

                # Print the bounding box coordinates
                rospy.loginfo(f"Bounding box coordinates: xmin={box[0]}, ymin={box[1]}, xmax={box[2]}, ymax={box[3]}")

                # Draw bounding box
                image = cv2.rectangle(image, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), color=(0, 255, 0), thickness=2)
                image = cv2.putText(image, "Raspberry", (int(box[0]), int(box[1] - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                # Print the center of the detected object
                rospy.loginfo(f"Center of detected object: x={middle[0]}, y={middle[1]}")

                shape_data = {
                    'bounding_box': {
                        'xmin': box[0],
                        'ymin': box[1],
                        'xmax': box[2],
                        'ymax': box[3]
                    },
                    'center': {
                        'x': middle[0],
                        'y': middle[1]
                    },
                    'pose': {
                        'x': pose[0],
                        'y': pose[1],
                        'z': pose[2]
                    }
                }
                detected_shapes.append(shape_data)

        # Publish detected shapes
        if detected_shapes:
            self.shape_pub.publish(json.dumps(detected_shapes))
            rospy.loginfo("Published detected shapes.")

        # Display the image with detections
        cv2.imshow("Detection", image)
        cv2.waitKey(1)
        rospy.loginfo("Displayed the image with detections.")


if __name__ == '__main__':
    try:
        ShapeDetectionNode()
    except rospy.ROSInterruptException:
        pass
