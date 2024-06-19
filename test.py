"""""""""
import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

prvotno = None

def callback(data):
    global prvotno

    # Extract the quaternion from the sensor data
    orientation = data.orientation
    quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)

    # Convert the quaternion to euler angles
    roll, pitch, yaw = euler_from_quaternion(quaternion)

    if prvotno is None:
        prvotno = yaw * 180.0 / 3.14159

    thing = yaw * 180.0 / 3.14159

    # Print the magnetic direction (yaw) to the console
    print("roll: {:.2f} degrees".format(roll))
    print("pitch: {:.2f} degrees".format(pitch))
    print("yaw: {:.2f} degrees".format(yaw))
    print("Trenutno: {:.2f} degrees".format(thing))
    print("Prvotno: {:.2f} degrees".format(prvotno))
    print("Cilj: {:.2f} degrees".format(abs((abs(prvotno) - 180))))
    print("Cilj: {:.2f} degrees".format(abs((abs(prvotno) +180))))

    if (round(abs(thing)) == round(abs((abs(prvotno) - 180))) or round(abs(thing)) == round(abs((abs(prvotno) + 180)))):
            rospy.signal_shutdown('Magnetic direction changed to opposite.')

def main():
    # Initialize the ROS node
    rospy.init_node('magnetic_direction_reader')

    # Subscribe to the IMU topic
    rospy.Subscriber('/imu/data', Imu, callback)

    # Run the ROS node until it's shut down
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass



"""""""""
import rospy
from sensor_msgs.msg import Imu
import tkinter as tk
import math

class SpeedGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Speed GUI")

        # Create a canvas for path visualization
        self.canvas_width = 1000
        self.canvas_height = 800
        self.canvas = tk.Canvas(self, width=self.canvas_width, height=self.canvas_height, bg="white")
        self.canvas.pack(pady=20)

        # Create a label for displaying the speed
        self.speed_label = tk.Label(self, text="Speed: 0 m/s", font=("Arial", 24))
        self.speed_label.pack()

        # Subscribe to the Imu topic
        rospy.init_node("speed_node")
        rospy.Subscriber("/imu/data", Imu, self.callback)

        # Store the previous speed, position, and heading angle
        self.previous_speed = 0.0
        self.position = (self.canvas_width / 2, self.canvas_height / 2)
        self.heading_angle = 0.0

        # Create a variable to hold the reference to the dot shape on the canvas
        self.dot = None

        # Low-pass filter parameters
        self.alpha = 0.2
        self.filtered_position = self.position

    def callback(self, data):
        acceleration = data.linear_acceleration
        accel_x = acceleration.x
        accel_y = acceleration.y
        accel_z = acceleration.z

        # Calculate the magnitude of the acceleration vector
        speed = math.sqrt((accel_x ** 2) + (accel_y ** 2) + (accel_z ** 2)) - 9.8

        # Check if the speed has changed significantly
        if abs(speed - self.previous_speed) > 0.05:
            # Update the speed label
            self.speed_label.config(text=f"Speed: {speed:.2f} m/s")

            # Update the position based on the speed and heading angle
            delta_x = speed * math.sin(math.radians(self.heading_angle))
            delta_y = speed * math.cos(math.radians(self.heading_angle))
            self.position = (self.position[0] + delta_x, self.position[1] - delta_y)

            # Apply low-pass filter to the position
            self.filtered_position = (
                self.alpha * self.filtered_position[0] + (1 - self.alpha) * self.position[0],
                self.alpha * self.filtered_position[1] + (1 - self.alpha) * self.position[1]
            )

            # Delete the previous dot shape, if it exists
            if self.dot:
                self.canvas.delete(self.dot)

            # Draw a red dot to represent the current position
            dot_radius = 5
            dot_x = self.filtered_position[0] - dot_radius
            dot_y = self.filtered_position[1] - dot_radius
            self.dot = self.canvas.create_oval(
                dot_x, dot_y, dot_x + dot_radius * 2, dot_y + dot_radius * 2, fill="red"
            )

            # Draw a line to represent the path
            self.canvas.create_line(
                self.filtered_position[0], self.filtered_position[1], self.filtered_position[0] + delta_x,
                self.filtered_position[1] - delta_y, fill="black"
            )

            # Update the heading angle based on the rotational motion
            rotation = data.angular_velocity.z  # Assuming the z-axis represents the rotational motion
            self.heading_angle += math.degrees(rotation)

            # Update the previous speed
            self.previous_speed = speed

    def run(self):
        # Start the main GUI loop
        self.mainloop()


if __name__ == '__main__':
    speed_gui = SpeedGUI()
    speed_gui.run()


"""""""""
import rospy
from sensor_msgs.msg import Imu
import tkinter as tk
import math
import matplotlib
matplotlib.use('tkagg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from tf.transformations import euler_from_quaternion


class SpeedGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Speed GUI")
        # Create a label for displaying the speed
        self.speed_label = tk.Label(self, text="Speed: 0 m/s", font=("Arial", 24))
        self.speed_label.pack(pady=20)

        # Create a 3D graph
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_xlim([-10, 10])
        self.ax.set_ylim([-10, 10])
        self.ax.set_zlim([-10, 10])

        # Store the previous speed and position
        self.previous_speed = 0.0
        self.previous_position = [0.0, 0.0, 0.0]

        # Subscribe to the Imu topic
        rospy.Subscriber("/imu/data", Imu, self.callback)

        # Start the GUI update process
        self.update_gui()

    def callback(self, data):
        acceleration = data.linear_acceleration
        accel_x = acceleration.x
        accel_y = acceleration.y
        accel_z = acceleration.z

        # Calculate the magnitude of the acceleration vector
        speed = math.sqrt((accel_x ** 2) + (accel_y ** 2) + (accel_z ** 2)) - 9.8

        # Check if the speed has changed significantly
        if abs(speed - self.previous_speed) > 0.05:
            # Update the previous speed
            self.previous_speed = speed

            # Get the current position
            x_pos, y_pos, z_pos = self.previous_position

            # Convert the quaternion to roll, pitch, and yaw angles
            orientation = data.orientation
            quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
            roll, pitch, yaw = euler_from_quaternion(quaternion)

            # Calculate the direction vector
            direction_x = speed * math.cos(pitch) * math.cos(yaw)
            direction_y = speed * math.sin(roll) * math.sin(pitch) * math.cos(yaw) - speed * math.cos(roll) * math.sin(
                yaw)
            direction_z = speed * math.cos(roll) * math.sin(pitch) * math.cos(yaw) + speed * math.sin(roll) * math.sin(
                yaw)

            # Calculate the new position based on speed and direction
            new_x_pos = x_pos + direction_x
            new_y_pos = y_pos + direction_y
            new_z_pos = z_pos + direction_z

            # Plot the line from previous position to new position
            self.ax.plot([x_pos, new_x_pos], [y_pos, new_y_pos], [z_pos, new_z_pos], c='b')

            # Set the new position as the previous position for the next update
            self.previous_position = [new_x_pos, new_y_pos, new_z_pos]

    def update_speed_label(self, speed):
        # Update the speed label
        self.speed_label.config(text=f"Speed: {speed:.2f} m/s")

    def update_gui(self):
        # Update the speed label
        self.update_speed_label(self.previous_speed)

        # Update the plot
        self.ax.clear()
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_xlim([-10, 10])
        self.ax.set_ylim([-10, 10])
        self.ax.set_zlim([-10, 10])
        x_pos, y_pos, z_pos = self.previous_position
        self.ax.plot([0, x_pos], [0, y_pos], [0, z_pos], c='b')
        self.fig.canvas.draw()

        # Schedule the next update
        self.after(100, self.update_gui)

    def run(self):
        plt.show()
        self.mainloop()


if __name__ == '__main__':
    rospy.init_node("speed_node")
    speed_gui = SpeedGUI()
    speed_gui.run()

"""""""""
"""""""""to je za prikaz hitrosti
import rospy
from sensor_msgs.msg import Imu
import tkinter as tk
import math

class SpeedGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Speed GUI")
        # Create a label for displaying the speed
        self.speed_label = tk.Label(self, text="Speed: 0 m/s", font=("Arial", 24))
        self.speed_label.pack(pady=20)

        # Subscribe to the Imu topic
        rospy.init_node("speed_node")
        rospy.Subscriber("/imu/data", Imu, self.callback)

        # Store the previous speed
        self.previous_speed = 0.0

    def callback(self, data):
        acceleration = data.linear_acceleration
        accel_x = acceleration.x
        accel_y = acceleration.y
        accel_z = acceleration.z

        # Calculate the magnitude of the acceleration vector
        speed = math.sqrt((accel_x**2) + (accel_y**2) + (accel_z**2)) - 9.8

        # Check if the speed has changed significantly
        if abs(speed - self.previous_speed) > 0.05:
            # Update the speed label
            self.speed_label.config(text=f"Speed: {speed:.2f} m/s")
            # Update the previous speed
            self.previous_speed = speed

    def run(self):
        # Start the main GUI loop
        self.mainloop()


if __name__ == '__main__':
    speed_gui = SpeedGUI()
    speed_gui.run()
"""""