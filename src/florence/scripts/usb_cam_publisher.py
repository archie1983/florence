#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage
from cv2 import *
import numpy as np

def usb_cam_publisher():
    pub = rospy.Publisher('camera1', CompressedImage, queue_size=10)
    rospy.init_node('usb_cam_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        #pub.publish(hello_str)

        cam = VideoCapture(0)   # 0 -> index of camera
        success, img = cam.read()
        if success:
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(imencode('.jpg', img)[1]).tostring()
            pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        usb_cam_publisher()
    except rospy.ROSInterruptException:
        pass
