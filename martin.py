import pyrealsense2 as rs

try:
    ctx = rs.context()
    devices = ctx.query_devices()
    device_count = len(devices)

    if device_count == 0:
        print("No devices connected.")
    else:
        for x in range(device_count):
            print(f"Device {x}: {devices[x].get_info(rs.camera_info.name)}")
except Exception as e:
    print(f"An error occurred: {e}")
