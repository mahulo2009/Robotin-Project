import rospy
import math

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


def callback(data):      
    orientation = [ data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w]
    (roll,pitch, yaw) = euler_from_quaternion(orientation)

    print(math.degrees(yaw))

if __name__ == "__main__":
    print("Hello Quaternion 2 Euler")

    rospy.init_node("quaternion2eluer")
    rospy.Subscriber("/raw_odom",Odometry,callback,queue_size=1)

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()