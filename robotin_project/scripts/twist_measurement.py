#!/usr/bin/python

import rospy
import threading
import numpy as np

from nav_msgs.msg import Odometry

vx_0=0
vx_1=0

def callback_measure():
    global vx_0,vx_1

    timer = threading.Timer(1,callback_measure)
    timer.start()
            
    print("Velocity:",vx_1)
    print("Acceleration:",vx_1-vx_0)

    vx_0 = vx_1

    print("---")

def callback_odom(data):
    global vx_1
    vx_1=data.twist.twist.linear.x


if __name__=="__main__":    
    rospy.init_node("odom_calibrate")
    rospy.Subscriber("/raw_odom",Odometry,callback_odom)

    callback_measure()

    rospy.spin()