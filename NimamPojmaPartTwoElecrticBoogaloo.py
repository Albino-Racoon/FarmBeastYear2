#gyroscope
import rospy
from sensor_msgs.msg import Imu
import tkinter as tk
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import matplotlib.pyplot as plt
import sys
sys.setrecursionlimit(10**8)



# Initialize gyro variables at the global level
gyro_x = None
gyro_y = None
gyro_z = None


# TO RUN: (venv) rakun@rakunComp:~/catkin_ws/src/farmbeast_simulation/src$ rosrun farmbeast_simulation test.py

def callback(data):
    global roll, pitch, yaw, gyro_x, gyro_y, gyro_z
    orientation_q = data.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    roll_label.config(text="Roll: {:.2f}".format(roll))
    pitch_label.config(text="Pitch: {:.2f}".format(pitch))
    yaw_label.config(text="Yaw: {:.2f}".format(yaw))

    gyro_x = data.angular_velocity.x
    gyro_y = data.angular_velocity.y
    gyro_z = data.angular_velocity.z
    gyro_x_label.config(text="Gyroscope X: {:.2f}".format(gyro_x))
    gyro_y_label.config(text="Gyroscope Y: {:.2f}".format(gyro_y))
    gyro_z_label.config(text="Gyroscope Z: {:.2f}".format(gyro_z))



def update_gui():
    global bar_plot, bar_plot_gyro
    root.after(100, update_gui)  # Update every 100 milliseconds
    root.update()

    # Clear the existing plot
    plt.clf()

    # Create a new bar plot with the current roll, pitch, and yaw values
    data = [roll, pitch, yaw]
    labels = ['Roll', 'Pitch', 'Yaw']
    bar_plot = plt.bar(labels, data)

    # Update the plot
    for i, rect in enumerate(bar_plot):
        height = rect.get_height()
        plt.text(rect.get_x() + rect.get_width() / 2.0, height, '{:.2f}'.format(data[i]), ha='center', va='bottom')

    # Create a new bar plot with the current gyroscope readings if they are not None
    if gyro_x is not None and gyro_y is not None and gyro_z is not None:
        data_gyro = [gyro_x, gyro_y, gyro_z]
        labels_gyro = ['Gyroscope X', 'Gyroscope Y', 'Gyroscope Z']
        bar_plot_gyro = plt.bar(labels_gyro, data_gyro)

        # Update the plot
        for i, rect in enumerate(bar_plot_gyro):
            height = rect.get_height()
            plt.text(rect.get_x() + rect.get_width() / 2.0, height, '{:.2f}'.format(data_gyro[i]), ha='center', va='bottom')

    plt.pause(0.001)


if __name__ == '__main__':
    rospy.init_node("imu_gui_node")  # Use a unique name for the node
    root = tk.Tk()
    root.title("IMU Data")

    # Create labels to display the
    roll_label = tk.Label(root, text="Roll: ")
    roll_label.pack()
    pitch_label = tk.Label(root, text="Pitch: ")
    pitch_label.pack()
    yaw_label = tk.Label(root, text="Yaw: ")
    yaw_label.pack()

    # Create labels to display the gyroscope readings
    gyro_x_label = tk.Label(root, text="   Gyroscope X: ")
    gyro_x_label.pack()
    gyro_y_label = tk.Label(root, text="   Gyroscope Y: ")
    gyro_y_label.pack()
    gyro_z_label = tk.Label(root, text="   Gyroscope Z: ")
    gyro_z_label.pack()

    rospy.Subscriber("/imu/data", Imu, callback)

    update_gui()
    root.mainloop()
