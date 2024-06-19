import rospy
from geometry_msgs.msg import Twist
import random

def publisher():
    pub = rospy.Publisher('stevilo_koruze_konec_vrste', Twist, queue_size=10)
    rospy.init_node('stevilo_koruze_konec_vrste', anonymous=True)
    rate = rospy.Rate(1)  # 1 Hz

    row_number = 1

    while not rospy.is_shutdown():
        if row_number > 10:  # Stop after publishing the 10th row
            # Publish a special message indicating the end
            twist = Twist()
            twist.linear.x = -1  # Special value to indicate stop
            twist.linear.y = -1  # Special value to indicate stop
            pub.publish(twist)
            break

        plant_count = random.randint(1, 100)  # Random plant count between 0 and 100

        twist = Twist()
        twist.linear.x = plant_count
        twist.linear.y = row_number

        rospy.loginfo(f"Publishing Row Number: {row_number}, Plant Count: {plant_count}")
        pub.publish(twist)

        row_number += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
