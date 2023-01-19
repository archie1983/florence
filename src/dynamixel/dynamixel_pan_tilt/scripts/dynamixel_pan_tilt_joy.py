#!/usr/bin/env python

import rospy
import math
from dynamixel_sdk import *
from dynamixel_sdk_examples.srv import *
from dynamixel_sdk_examples.msg import *
from dynamixel_pan_tilt.msg import PanTiltAngle
from sensor_msgs.msg import Joy

pub_topic = "/pan_tilt_joy"
pub = rospy.Publisher(pub_topic, PanTiltAngle, queue_size=1)

k = 4095 / 360
dxl_zero_pos = 2048
tilt_multiplier = 1.2
tilt_offset = 20

def set_goal_angle_callback(data):

    pta = PanTiltAngle()

    if (abs(data.axes[6]) > 0.5 or abs(data.axes[7]) > 0.5):
        pan_angle, tilt_angle = get_dyna_angles()

        #global pub
        if (data.axes[6] > 0.5):
            pta.pan_angle = pan_angle - 2
            pta.tilt_angle = tilt_angle

        if (data.axes[6] < -0.5):
            pta.pan_angle = pan_angle + 2
            pta.tilt_angle = tilt_angle

        if (data.axes[7] > 0.5):
            pta.pan_angle = pan_angle
            pta.tilt_angle = tilt_angle - 2

        if (data.axes[7] < -0.5):
            pta.pan_angle = pan_angle
            pta.tilt_angle = tilt_angle + 2
        
        rospy.loginfo("Dynamixel 1 angle: %f, Dynamixel 2 angle: %f", pta.pan_angle, pta.tilt_angle)
        if (pta.pan_angle > -160 and pta.tilt_angle > -160):
            pub.publish(pta)
        


def get_dyna_angles():
    get_pos_service = rospy.get_param('get_pos_service', '/sync_get_position')
    pos_serv = rospy.ServiceProxy(get_pos_service, SyncGetPosition)
    
    pan_angle = 0
    tilt_angle = 0
    try:
        res = SyncGetPositionRequest()
        res.id1 = 11
        res.id2 = 12
        dxl_pos = pos_serv(res)
        rospy.loginfo("Dynamixel 1 pos: %i, Dynamixel 2 pos: %i", dxl_pos.position1, dxl_pos.position2)
        pan_angle = dxl_position_to_angle(dxl_pos.position1, flip = -1)
        tilt_angle = dxl_position_to_angle(dxl_pos.position2, multiplier=tilt_multiplier, offset=tilt_offset)
    except rospy.ServiceException as e:
        rospy.loginfo("dyna position request failed")
    return pan_angle, tilt_angle



def pan_tilt_joy_node():
    rospy.init_node('pan_tilt_joy_node')
    joy_topic = rospy.get_param('~joy_topic', 'joy')
    rospy.Subscriber(joy_topic, Joy, set_goal_angle_callback)
    rospy.spin()

def clamp(n, smallest, largest): return max(smallest, min(n, largest))

def angle_to_dxl_position(angle, flip = 1, multiplier = 1, offset = 0):
    return int(math.floor(dxl_zero_pos + flip * k * (multiplier * angle + offset)))

def dxl_position_to_angle(position, flip = 1, multiplier = 1, offset = 0):
    angle = (position - dxl_zero_pos) / (flip * k * multiplier) - offset / multiplier
    return angle


def main():
    pan_tilt_joy_node()


if __name__ == '__main__':
    main()
