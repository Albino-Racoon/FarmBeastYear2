import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

# Path to the ROS bag file
bag_file = r"/home/rakun/catkin_ws/src/farmbeast_simulation/src/FB_uporabno/2024-06-10-20-41-30.bag"
# Topic name containing the images
image_topic = '/your_correct_topic_name'
# Output directory for extracted frames
output_dir = 'frames'
os.makedirs(output_dir, exist_ok=True)

bag = rosbag.Bag(bag_file, 'r')
bridge = CvBridge()

count = 0
for topic, msg, t in bag.read_messages(topics=[image_topic]):
    try:
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        frame_path = os.path.join(output_dir, f'frame_{count:04d}.jpg')
        cv2.imwrite(frame_path, cv_img)
        count += 1
    except Exception as e:
        print(f"Error processing frame {count}: {e}")

bag.close()
print(f'Extracted {count} frames from {bag_file}')
