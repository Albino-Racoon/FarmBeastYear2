#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32MultiArray
import cv2
from pathlib import Path
import sys
import numpy as np

# Ensure YOLOv8 is in the system path
sys.path.insert(0, str(Path(__file__).resolve().parent / 'yolov8'))

# Import YOLOv8
from ultralytics import YOLO

# Initialize YOLO model
model = YOLO('/home/rakun/Downloads/maline1.pt')  # Update this to the path of your YOLOv8 model

bridge = CvBridge()

# Initialize ROS node
rospy.init_node('yolo_node')

# Publisher for transformed center coordinates
transformed_center_pub = rospy.Publisher('/transformed_center', Float32MultiArray, queue_size=10)

# Global variables to store the images
rgb_image = None
depth_image = None

# Camera intrinsics (example values, you need to use your camera's intrinsics)
fx = 617.571044921875  # Focal length in pixels (x-axis)
fy = 617.571044921875  # Focal length in pixels (y-axis)
cx = 322.0910949707031  # Principal point (x-axis)
cy = 241.14906311035156  # Principal point (y-axis)

def image_callback(msg):
    global rgb_image
    print("RGB Image received")
    try:
        # Convert ROS Image message to OpenCV image
        rgb_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Save the image to debug
        cv2.imwrite('/tmp/rgb_image_debug.png', rgb_image)
    except CvBridgeError as e:
        print(f"Error converting RGB image: {e}")
    process_images()

def depth_callback(msg):
    global depth_image
    print("Depth Image received")
    try:
        # Convert ROS Image message to OpenCV image
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        # Save the depth image to debug
        depth_debug_image = (depth_image / np.max(depth_image) * 255).astype(np.uint8)
        cv2.imwrite('/tmp/depth_image_debug.png', depth_debug_image)
    except CvBridgeError as e:
        print(f"Error converting depth image: {e}")
    process_images()

def process_images():
    global rgb_image, depth_image

    if rgb_image is None or depth_image is None:
        return

    print("Processing images...")

    # Ensure the frame is writable
    frame = rgb_image.copy()

    # Prepare image for YOLO
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = model(img)

    # Get image dimensions
    height, width, _ = frame.shape

    # Draw grid on the image
    grid_size = 25
    cell_width = width // grid_size
    cell_height = height // grid_size

    for i in range(grid_size + 1):
        # Draw vertical lines
        cv2.line(frame, (i * cell_width, 0), (i * cell_width, height), (255, 255, 255), 1)
        # Draw horizontal lines
        cv2.line(frame, (0, i * cell_height), (width, i * cell_height), (255, 255, 255), 1)

    # Define real-world coordinates for grid corners
    real_coords = np.zeros((grid_size + 1, grid_size + 1, 2))

    # Bottom row
    for i in range(grid_size + 1):
        real_coords[0, i, 0] = -45 + i * (90 / grid_size)
        real_coords[0, i, 1] = 150

    # Top row
    for i in range(grid_size + 1):
        real_coords[-1, i, 0] = -45 + i * (90 / grid_size)
        real_coords[-1, i, 1] = 350

    # Left column
    for i in range(grid_size + 1):
        real_coords[i, 0, 0] = -45
        real_coords[i, 0, 1] = 150 + i * (200 / grid_size)

    # Right column
    for i in range(grid_size + 1):
        real_coords[i, -1, 0] = 45
        real_coords[i, -1, 1] = 150 + i * (200 / grid_size)

    # Interpolate other points
    for i in range(1, grid_size):
        for j in range(1, grid_size):
            real_coords[i, j, 0] = real_coords[0, j, 0] + (real_coords[-1, j, 0] - real_coords[0, j, 0]) * (i / grid_size)
            real_coords[i, j, 1] = real_coords[i, 0, 1] + (real_coords[i, -1, 1] - real_coords[i, 0, 1]) * (j / grid_size)

    # Draw detections
    for result in results:
        for box in result.boxes:
            xyxy = box.xyxy[0].cpu().numpy()  # Ensure array is on CPU
            conf = box.conf[0].cpu().numpy()  # Ensure array is on CPU
            cls = box.cls[0].cpu().numpy()    # Ensure array is on CPU
            label = f'{model.names[int(cls)]} {conf:.2f}'
            plot_one_box(xyxy, frame, label=label, color=(0, 255, 0))

            # Calculate the center of the bounding box
            center_x = (xyxy[0] + xyxy[2]) / 2
            center_y = (xyxy[1] + xyxy[3]) / 2

            # Ensure the center coordinates are within the depth image dimensions
            if center_y >= depth_image.shape[0] or center_x >= depth_image.shape[1]:
                continue

            # Get the depth value at the center of the bounding box
            depth_value = depth_image[int(center_y), int(center_x)]

            # Print the coordinates of the center of the bounding box for detected flowers
            if model.names[int(cls)] == 'Flower':
                grid_x = int(center_x // cell_width)
                grid_y = int(center_y // cell_height)
                grid_center_x = (grid_x * cell_width) + (cell_width / 2)
                grid_center_y = (grid_y * cell_height) + (cell_height / 2)
                print(f"Flower is in grid cell: ({grid_x}, {grid_y}), Center of grid cell: ({grid_center_x}, {grid_center_y})")

                # Transform to real-world coordinates
                transformed_x = real_coords[grid_size - grid_y, grid_x, 0]
                transformed_y = real_coords[grid_size - grid_y, grid_x, 1]
                transformed_z = depth_value  # Using depth as the z coordinate

                # Calculate distance
                distance = depth_value

                # Calculate angles
                angle_x = np.arctan((center_x - cx) / fx)
                angle_y = np.arctan((center_y - cy) / fy)

                print(f"Transformed center: ({transformed_x}, {transformed_y}, {transformed_z})")
                print(f"Distance: {distance} meters, Angle X: {np.degrees(angle_x)} degrees, Angle Y: {np.degrees(angle_y)} degrees")

                # Publish the transformed coordinates
                transformed_center = Float32MultiArray(data=[transformed_x, transformed_y, transformed_z])
                transformed_center_pub.publish(transformed_center)

    # Display the frame
    cv2.imshow('YOLO Detections', frame)
    cv2.waitKey(1)

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
    print("Starting YOLO node")
    image_sub = rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, depth_callback)

    print("Spinning")
    rospy.spin()
    cv2.destroyAllWindows()
