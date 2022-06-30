#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
rospy.init_node('florence_mover')
rate = rospy.Rate(10) # 2hz

def florence_mover():
    move = Twist()
    move.angular.z = 0.5

    pub.publish(move)

    while not rospy.is_shutdown():
        pub.publish(move)
        rate.sleep()

if __name__ == '__main__':
    try:
        florence_mover()
    except rospy.ROSInterruptException:
        pass
