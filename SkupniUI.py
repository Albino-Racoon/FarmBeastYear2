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

class SpeedDisplay:
    #
    def __init__(self, master):
        self.speed_label = tk.Label(master, text=" ")
        self.speed_label.pack(side=tk.TOP)

    def update_speed(self, speed):
        self.speed_label.config(text=" ")

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
        # Calculate speed from the gyroscope readings
        speed = math.sqrt(gyro_x ** 2 + gyro_y ** 2 + gyro_z ** 2)

        # Update the speed display
        speed_display.update_speed(speed)

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
        self.ax.set_title('IMU Pot')
        plt.draw()

class PygameFrame:
    def __init__(self, master):
        self.master = master
        self.canvas = tk.Canvas(master, width=500, height=500)
        self.canvas.pack(side=tk.LEFT)
        self.draw_pygame_frame()

    def draw_pygame_frame(self):
        # Initialize Pygame
        pygame.init()

        # Create a Pygame surface
        surface = pygame.Surface((500, 500))

        # Draw something on the surface
        pygame.draw.circle(surface, (255, 0, 0), (250, 250), 50)

        # Convert the Pygame surface to a Tkinter PhotoImage
        image = pygame.surfarray.array3d(surface)
        photo = tk.PhotoImage(master=self.canvas, data=image)

        # Draw the PhotoImage on the canvas
        self.canvas.create_image(0, 0, image=photo, anchor=tk.NW)

        # Schedule the next update
        self.master.after(100, self.draw_pygame_frame)

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

class Compass(tk.Canvas):
    def __init__(self, master=None, **kw):
        super().__init__(master, **kw)
        self._size = min(self.winfo_reqwidth(), self.winfo_reqheight())
        self.configure(width=self._size, height=self._size)
        self._needle = self.create_line(0, 0, 0, 0, width=0.0000000000005, fill="gray")
        self._heading = 0
        self.draw()
        self.delete()

    def set_heading(self, heading):
        self._heading = heading
        self.draw_needle()

    def draw_needle(self):
        if self._needle is not None:
            self.delete(self._needle)
        r = self._size / 2
        angle = np.radians(-self._heading + 90)
        x1 = r
        y1 = r
        x2 = r + r * 0.8 * np.cos(angle)
        y2 = r + r * 0.8 * np.sin(angle)
        self._needle = self.create_line(x1, y1, x2, y2, width=5, fill="red")

    def draw(self):
        # Clear the existing drawing
        self.delete('all')

        # Draw the outer circle
        r = self._size / 2
        self.create_oval(r / 10, r / 10, r * 1.9, r * 1.9, width=2)

        # Draw the cardinal directions
        # directions = [' Sever ', ' Vzhod ', ' Jug ', ' Zahod ']
        directions = [' Vzhod                  ', ' Jug \n\n', '               Zahod ', ' \n \n Sever ']
        for i, direction in enumerate(directions):
            angle = np.radians(i * 90)
            x = r * np.cos(angle) + r
            y = r * np.sin(angle) + r
            self.create_text(x, y, text=direction)

        # Remove the previous needle
        self.delete("needle")

        # Draw the new compass needle
        angle = np.radians(-self._heading + 90)
        x1 = r
        y1 = r
        x2 = r + r * 0.8 * np.cos(angle)
        y2 = r + r * 0.8 * np.sin(angle)
        self.create_line(x1, y1, x2, y2, width=5, tags=("needle",))




if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node("compass_and_imu_gui_node")

    # Create the main window
    root = tk.Tk()
    root.title("Compass in IMU Podatki")
    # Create a frame to hold the IMU data labels and graph
    imu_frame = tk.Frame(root)
    imu_frame.pack(side=tk.LEFT)

    # Create labels to display the orientation data
    roll_label = tk.Label(imu_frame, text="Roll: ")
    roll_label.pack(side=tk.TOP)
    pitch_label = tk.Label(imu_frame, text="Pitch: ")
    pitch_label.pack(side=tk.TOP)
    yaw_label = tk.Label(imu_frame, text="Yaw: ")
    yaw_label.pack(side=tk.TOP)

    # Create labels to display the gyroscope readings
    gyro_x_label = tk.Label(imu_frame, text="Gyroscope X: ")
    gyro_x_label.pack(side=tk.TOP)
    gyro_y_label = tk.Label(imu_frame, text="Gyroscope Y: ")
    gyro_y_label.pack(side=tk.TOP)
    gyro_z_label = tk.Label(imu_frame, text="Gyroscope Z: ")
    gyro_z_label.pack(side=tk.TOP)
    gyro_speed_label = tk.Label(imu_frame, text="Hitrost: ")
    gyro_speed_label.pack(side=tk.TOP)

    fig2 = plt.Figure(figsize=(5, 5), dpi=100)
    ax2 = fig2.add_subplot(111)
    bar = ax2.bar(['X', 'Y', 'Z'], [5, 5, 5])

    # Create a canvas to draw the compass
    canvas_width = 250
    canvas_height = 250
    canvas = Compass(root, width=canvas_width, height=canvas_height)
    canvas.pack(side=tk.LEFT)

    canvas3 = FigureCanvasTkAgg(fig2, master=imu_frame)
    canvas3.draw()
    canvas3.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

    fig3 = plt.Figure(figsize=(5, 5), dpi=100)
    ax3 = fig3.add_subplot(111, projection='3d')
    imu_plot = IMUPlot(fig3, ax3)


    # Create a canvas for the third graph
    canvas4 = FigureCanvasTkAgg(fig3, master=root)
    canvas4.draw()
    canvas4.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

    # Create a canvas for the fourth graph
    canvas5 = tk.Canvas(root, width=500, height=500)
    canvas5.pack(side=tk.BOTTOM)










    # Create a figure for the graph
    fig = plt.Figure(figsize=(5, 4), dpi=100)
    ax = fig.add_subplot(111)
    ax.set_xlabel('Čas')
    ax.set_ylabel('Podatki')
    line, = ax.plot([], [])
    # Create a canvas for the graph
    canvas2 = FigureCanvasTkAgg(fig, master=imu_frame)
    canvas2.draw()
    canvas2.get_tk_widget().pack(side=tk.RIGHT, fill=tk.BOTH, expand=1)

    # Create a toolbar for the graph
    toolbar = NavigationToolbar2Tk(canvas2, imu_frame)
    toolbar.update()
    canvas2.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH, expand=1)

    # Create the SpeedDisplay object
    speed_display = SpeedDisplay(imu_frame)



    """""""""
    # Create a frame for the Pygame window
    pygame_frame = ttk.Frame(root, width=500, height=500)
    pygame_frame.pack(side=tk.RIGHT)

    # Create the Pygame window
    PygameFrame(pygame_frame)
        """""""""""
    def update_gui():
        # Get the latest IMU data
        data = rospy.wait_for_message("/imu/data", Imu, timeout=0.1)
        if data:
            # Call the callback method of the IMUPlot object
            imu_plot.callback(data)

        orientation_q = data.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        # Update the compass heading
        canvas.set_heading(np.degrees(yaw))
        # Extract speed from the gyroscope readings
        speed = math.sqrt(data.angular_velocity.x ** 2 + data.angular_velocity.y ** 2 + data.angular_velocity.z ** 2)

        # Update the labels for the orientation and gyroscope readings
        roll_label.config(text="Roll: {:.2f}".format(np.degrees(roll)))
        pitch_label.config(text="Pitch: {:.2f}".format(np.degrees(pitch)))
        yaw_label.config(text="Yaw: {:.2f}".format(np.degrees(yaw)))
        gyro_x_label.config(text="Gyroscope X: {:.2f}".format(data.angular_velocity.x))
        gyro_y_label.config(text="Gyroscope Y: {:.2f}".format(data.angular_velocity.y))
        gyro_z_label.config(text="Gyroscope Z: {:.2f}".format(data.angular_velocity.z))
        acceleration = data.linear_acceleration
        accel_x = acceleration.x
        accel_y = acceleration.y
        accel_z = acceleration.z
        speed = math.sqrt((accel_x ** 2) + (accel_y ** 2) + (accel_z ** 2)) - 9.8
       ## if abs(speed - self.previous_speed) > 0.05:
            # Update the speed label

        gyro_speed_label.config(text=f"Hitrost: {abs(speed):.2f} m/s")

        # Update the speed display
        speed_display.update_speed(speed)
        # Add the gyroscope reading to the graph
        line.set_xdata(np.append(line.get_xdata(), time.time()))
        line.set_ydata(np.append(line.get_ydata(), data.angular_velocity.x))
        ax.relim()
        ax.autoscale_view()
        canvas2.draw()

        bar[0].set_height(data.angular_velocity.x)
        bar[1].set_height(data.angular_velocity.y)
        bar[2].set_height(data.angular_velocity.z)
        canvas3.draw()

        #imu_plot.callback(data)
        imu_plot.update_plot(data)
        canvas4.draw()

        # Schedule the next update
        root.after(100, update_gui)

    # Start the GUI update loop
    update_gui()

    # Start the main Tkinter event loop
    root.mainloop()

