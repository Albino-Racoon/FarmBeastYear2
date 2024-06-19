# LIVE YOLOv8 WITH DEPTH FILTERS
from IPython.display import clear_output
import pyrealsense2 as rs
import numpy as np
import cv2
import logging
from ultralytics import YOLO
from PIL import Image as im

model_path = "/home/rakun/Downloads/maline1.pt"

cameras = {
    'D435': '241222074803'
}

model = YOLO(model_path)  # load a custom model

from_edge = 100
image_width = 1280
image_height = 720

# Configure depth and color streams from Camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_device(cameras['D435'])  # D435
config.enable_stream(rs.stream.depth, image_width, image_height, rs.format.z16, 30)
config.enable_stream(rs.stream.color, image_width, image_height, rs.format.bgr8, 30)

# Start streaming
cfg = pipeline.start(config)

align_to = rs.stream.color
align = rs.align(align_to)

x = 0
dist_z = np.empty((image_height, image_width))

while True:

    # Camera intrinsics
    profile = cfg.get_stream(rs.stream.depth)  # Fetch stream profile for depth stream
    intr = profile.as_video_stream_profile().get_intrinsics()  # Downcast to video_stream_profile and fetch intrinsics

    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    if not depth_frame or not color_frame:
        continue

    # FILTERS
    depth_to_disparity = rs.disparity_transform(True)
    disparity_to_depth = rs.disparity_transform(False)

    decimation = rs.decimation_filter()
    decimation.set_option(rs.option.filter_magnitude, 1)

    spatial = rs.spatial_filter()
    spatial.set_option(rs.option.filter_magnitude, 5)
    spatial.set_option(rs.option.filter_smooth_alpha, 1)
    spatial.set_option(rs.option.filter_smooth_delta, 50)
    spatial.set_option(rs.option.holes_fill, 3)  # basic spatial hole filling capabilities

    temporal = rs.temporal_filter()

    hole_filling = rs.hole_filling_filter()

    frame = decimation.process(depth_frame)
    frame = spatial.process(frame)
    frame = temporal.process(frame)
    frame = hole_filling.process(frame)

    depth_frame = frame.as_depth_frame()

    # Convert images to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    yolo_image = color_image

    # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.3), cv2.COLORMAP_JET)

    # Stack all images horizontally
    images = np.hstack((color_image, depth_colormap))
    # images = np.vstack((color_image, depth_colormap))

    # cv2.waitKey(100)
    results = model(color_image)
    for result in results:
        boxes = result.boxes.data.cpu().numpy()  # Boxes object for bounding box outputs
        # result.show()  # display to screen

    for j in range(5):
        boxes = np.vstack([boxes, [0, 0, 0, 0, 0, 0]])

        # if (len(boxes)) != 0:
    for i in range(len(boxes[0]) - 1):

        if boxes[i, 0] < from_edge or boxes[i, 2] > image_width - from_edge or boxes[i, 1] < from_edge or boxes[
            i, 1] > image_height - from_edge:
            continue
        else:
            middle = [boxes[i, 0] + (boxes[i, 2] - boxes[i, 0]) / 2, boxes[i, 1] + (boxes[i, 3] - boxes[i, 1]) / 2]

            yolo_image = cv2.circle(yolo_image, (int(middle[0]), int(middle[1])), radius=3, color=(255, 0, 0),
                                    thickness=-1)
            yolo_image = cv2.rectangle(yolo_image, (int(boxes[i, 0]), int(boxes[i, 1])),
                                       (int(boxes[i, 2]), int(boxes[i, 3])), color=(0, 255, 0), thickness=2)

            # if middle[0] != 0.0 or middle[1] != 0.0:
            pose = rs.rs2_deproject_pixel_to_point(intr, [int(middle[0]), int(middle[1])],
                                                   depth_frame.get_distance(int(middle[0]), int(middle[1])))
            pose = [round(i * 1000, 1) for i in pose]
            yolo_image = cv2.putText(yolo_image, f"x={round(pose[0], 1)}, y={round(pose[1], 1)}, z={round(pose[2], 1)}",
                                     (int(boxes[i, 0]), int(boxes[i, 1] - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                     (255, 255, 255), 1)

        # if pose[2] < 350 and pose[2] != 0.0:
        #     print(pose)
        #     cv2.waitKey(0)
    # print(boxes[0,0])
    # print(boxes[0,1])
    # print(boxes[0,2])
    # print(boxes[0,3])

    cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
    cv2.imshow("Image", yolo_image)
    # cv2.waitKey(0)

    # # Show images of camera
    # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    # #cv2.namedWindow('RealSense', cv2.WINDOW_NORMAL)
    # cv2.imshow('RealSense', images)
    # #cv2.waitKey(1)

    # Simulate robot picking when pressed 's' key
    ch = cv2.waitKey(25)
    if ch == 115:
        # Get depth
        print(f"Picked {x}")

    # Press esc or 'q'  to close the image window
    if cv2.getWindowProperty('Image',
                             cv2.WND_PROP_VISIBLE) < 1:  # cv2.getWindowProperty('RealSense', cv2.WND_PROP_VISIBLE) <1
        cv2.destroyAllWindows()
        break
    clear_output(wait=True)
# Stop streaming
pipeline.stop()