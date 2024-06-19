
#attempt za korekcijo lidarja z uporabo podatkov nagiba  iz imu senzorja
"""""""""
import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import math

initial_yaw = None

def callback(data):
    global initial_yaw

    # Extract the quaternion from the sensor data
    orientation = data.orientation
    quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)

    # Convert the quaternion to euler angles
    roll, pitch, yaw = euler_from_quaternion(quaternion)

    if initial_yaw is None:
        initial_yaw = yaw

    # Calculate the angle required to move back to the initial position
    angle_diff = math.degrees(initial_yaw - yaw)
    if angle_diff < -180:
        angle_diff += 360
    elif angle_diff > 180:
        angle_diff -= 360

    # Calculate the vector required to move back to the initial position
    vector_diff = [math.cos(yaw), math.sin(yaw)]

    # Print the roll, pitch, and yaw values
    print("Roll: {:.2f} degrees".format(math.degrees(roll)))
    print("Pitch: {:.2f} degrees".format(math.degrees(pitch)))
    print("Yaw: {:.2f} degrees".format(math.degrees(yaw)))

    # Print the angle and vector required to move back to the initial position
    print("Razlika od začetne vrednosti: {:.2f} degrees".format(angle_diff))#fora je da da druge vrednosti sam ko skos laufa
    print("Kok se more premaknt da bo nazaj v originalni poziciji (Glede na  rotacijo roll-a, pitch-a pa yaw-a): [{:.2f}, {:.2f}, {:.2f}]".format(-roll* 180.0 / 3.14159, -pitch* 180.0 / 3.14159, -yaw* 180.0 / 3.14159))

    if round(abs(angle_diff)) == 0:
        rospy.signal_shutdown('Sensor returned to the initial position.')

def main():
    # Initialize the ROS node
    rospy.init_node('magnetic_direction_reader')

    # Subscribe to the IMU topic
    rospy.Subscriber('/imu/data', Imu, callback)

    # Run the ROS node until it's shut down
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass



"""""""""

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
"""""""""""""""
import math
import time
from OpenGL.GL import *
from OpenGL.GLU import *
import rospy
import sys
import tkinter as tk
import pygame
from OpenGL.GL import glEnd
from OpenGL.raw.GL.VERSION.GL_1_0 import glVertex3f
from matplotlib.backends._backend_tk import NavigationToolbar2Tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import numpy as np
from tf.transformations import euler_from_quaternion
from twisted.names import root
import tkinter as tk
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import Axes3D

from tkinter import ttk
from tkinter import *
import matplotlib.animation as animation
sys.setrecursionlimit(10**8)


# Initialize gyro variables at the global level
gyro_x = None
gyro_y = None
gyro_z = None

class IMUPlot:
    def __init__(self, fig, ax):
        self.x = [0]
        self.y = [0]
        self.z = [0]
        self.last_gyro = np.array([0.0, 0.0, 0.0])
        self.last_time = None
        self.ax = ax
        self.ax.plot(self.x, self.y, self.z)
        self.ax.set_xlabel('X position')
        self.ax.set_ylabel('Y position')
        self.ax.set_zlabel('Z position')
        self.ax.set_title('IMU path')

    def callback(self, data):
        gyro_x = data.angular_velocity.x
        gyro_y = data.angular_velocity.y
        gyro_z = data.angular_velocity.z

        # Calculate time difference between current and last measurement
        current_time = rospy.Time.now()
        if self.last_time is None:
            time_diff = rospy.Duration.from_sec(0.0)
        else:
            time_diff = current_time - self.last_time

        # Calculate displacement in x, y, and z directions
        displacement_x = (gyro_x + self.last_gyro[0]) / 2 * time_diff.to_sec()
        displacement_y = (gyro_y + self.last_gyro[1]) / 2 * time_diff.to_sec()
        displacement_z = (gyro_z + self.last_gyro[2]) / 2 * time_diff.to_sec()

        # Update last gyro and time
        self.last_gyro = np.array([gyro_x, gyro_y, gyro_z])
        self.last_time = current_time

        # Check if the robot has moved
        if abs(displacement_x) < 0.0001 and abs(displacement_y) < 0.0001 and abs(displacement_z) < 0.0001:
            return

        # Update x, y, and z positions
        self.x.append(self.x[-1] + displacement_x)
        self.y.append(self.y[-1] + displacement_y)
        self.z.append(self.z[-1] + displacement_z)

    def update_plot(self, num):
        self.x = self.x[-1000:]  # truncate to the same length
        self.y = self.y[-1000:]
        self.z = self.z[-1000:]
        self.ax.cla()
        self.ax.plot(self.x, self.y, self.z)
        self.ax.set_xlabel('X position')
        self.ax.set_ylabel('Y position')
        self.ax.set_zlabel('Z position')
        self.ax.set_title('IMU path')

class GUI:
    def __init__(self, master):
        self.master = master
        self.master.title('IMU Data')
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.imu_plot = IMUPlot(self.fig, self.ax)

        # Create a Pygame frame
        self.frame = tk.Frame(self.master, width=640, height=480)
        self.frame.pack()

        # Initialize Pygame
        pygame.init()
        self.screen = pygame.display.set_mode((640, 480), pygame.OPENGL | pygame.DOUBLEBUF)
        pygame.display.set_caption('Compass Display')
        glEnable(GL_DEPTH_TEST)

        # Initialize compass variables
        self.compass_angle = 0

        # Set up Pygame clock
        self.clock = pygame.time.Clock()

        # Set up ROS subscriber
        rospy.init_node('imu_subscriber')
        rospy.Subscriber('/imu_topic', Imu, self.imu_callback)

    def imu_callback(self, data):
        # Extract Euler angles from quaternion
        orientation = data.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        euler_angles = euler_from_quaternion(quaternion)
        roll = euler_angles[0]
        pitch = euler_angles[1]
        yaw = euler_angles[2]

        # Update compass angle
        self.compass_angle = math.degrees(yaw)

    def draw_compass(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()

        # Set up camera
        gluPerspective(45, 640 / 480, 0.1, 50.0)
        glTranslatef(0.0, 0.0, -5)

        # Draw compass needle
        glRotatef(-self.compass_angle, 0.0, 0.0, 1.0)
        glBegin(GL_LINES)
        glColor3f(1.0, 0.0, 0.0)
        glVertex3f(0, 0, 0)
        glVertex3f(0, 0.75, 0)
        glEnd()

        pygame.display.flip()

    def run(self):
        while not rospy.is_shutdown():
            # Handle Pygame events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

            self.frame.update_idletasks()
            self.frame.update()

            # Update the Pygame frame
            self.draw_compass()
            self.clock.tick(60)

            # Update the IMU plot
            self.imu_plot.update_plot(0)

            plt.show()

# Create a Tkinter window
root = tk.Tk()
app = GUI(root)
root.mainloop()

"""""""""