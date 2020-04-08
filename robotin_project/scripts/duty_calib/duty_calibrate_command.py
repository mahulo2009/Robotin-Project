#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from robotin_project.msg import TEL

pub = rospy.Publisher('/car/cmd_vel', Twist, queue_size=10)

wheel_radious = 0.034
max_dutty = 255

velocity = np.arange(0.0,wheel_radious*max_dutty,step=0.5)
point_per_velocity = 20

def cmd():
    rate = rospy.Rate(1) # 1hz
    twist = Twist()

    for v in velocity:
      for c in range(point_per_velocity):        
        twist.linear.x=v
        pub.publish(twist)            
        rate.sleep()      

if __name__ == '__main__':
  try:
    rospy.init_node('duty_calib', anonymous=True)
    cmd()
    
  except rospy.ROSInterruptException:
    pass