#!/usr/bin/python

import rospy 
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

pub = rospy.Publisher("/imu",Imu,queue_size=10)

def callback(data):
    global pub 

    header = Header()
    header.frame_id="imu"  
    header.stamp = rospy.Time.now() 

    data.header = header


    data.orientation_covariance = (0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025)
    data.angular_velocity_covariance = (0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025)
    data.linear_acceleration_covariance = (0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025)

    pub.publish(data)
    

    

if __name__ == "__main__":
    print("Hello Imu!")

    rospy.init_node("imu_transform")
    rospy.Subscriber("/android/imu",Imu,callback)


    rospy.spin()
