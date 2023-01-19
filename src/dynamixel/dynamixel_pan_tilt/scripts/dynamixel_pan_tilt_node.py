#!/usr/bin/env python3

#*******************************************************************************
# Copyright 2021 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#*******************************************************************************

#*******************************************************************************
# This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
# For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
# To test this example, please follow the commands below.
#
# Open terminal #1
# $ roscore
#
# Open terminal #2
# $ rosrun dynamixel_sdk_examples read_write_node.py
#
# Open terminal #3 (run one of below commands at a time)
# $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 1, position: 0}"
# $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 1, position: 1000}"
# $ rosservice call /get_position "id: 1"
#
# Author: Will Son
#******************************************************************************/

import os
import rospy
import math
from dynamixel_sdk import *
from dynamixel_sdk_examples.srv import *
from dynamixel_sdk_examples.msg import *
from dynamixel_pan_tilt.msg import PanTiltAngle

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

# Control table address
ADDR_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_GOAL_POSITION      = 116
ADDR_PRESENT_POSITION   = 132

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID                     = 11                # Pan Dynamixel ID
DXL2_ID	                    = 12		# Tilt Dynamixel ID
BAUDRATE                    = 1000000             # Dynamixel baudrate
DEVICENAME                  = rospy.get_param('~port', '/dev/dynamixel')

TORQUE_ENABLE = 1                 		# Value for enabling the torque
TORQUE_DISABLE = 0                		# Value for disabling the torque
DXL1_MINIMUM_POSITION_VALUE  = 0               	# Pan Dynamixel will rotate between this value
DXL1_MAXIMUM_POSITION_VALUE  = 4095            	# and this value

DXL2_MINIMUM_POSITION_VALUE  = 1024             # Dynamixel will rotate between this value
DXL2_MAXIMUM_POSITION_VALUE  = 3072            	# and this value
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

node_name = rospy.get_param('node_name', 'pan_tilt_node')

k = 4095 / 360
dxl_zero_pos = 2048

# Add offset and multiplier to tilt to make it easier to look down at robot
tilt_offset = 0
tilt_multiplier = 1.2

def set_goal_pos_callback(data):
    pan_dxl_pos = angle_to_dxl_position(data.pan_angle, flip=-1)
    tilt_dxl_pos = angle_to_dxl_position(data.tilt_angle, multiplier=tilt_multiplier, offset=tilt_offset)

    pan_dxl_pos = clamp(pan_dxl_pos, DXL1_MINIMUM_POSITION_VALUE, DXL1_MAXIMUM_POSITION_VALUE)
    tilt_dxl_pos = clamp(tilt_dxl_pos, DXL2_MINIMUM_POSITION_VALUE, DXL2_MAXIMUM_POSITION_VALUE)

    #print ("Pan: %s, Tilt: %s" % (pan_dxl_pos, tilt_dxl_pos))
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, pan_dxl_pos)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_POSITION, tilt_dxl_pos)

def sync_get_present_pos(req):
    dxl1_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, req.id1, ADDR_PRESENT_POSITION)
    dxl2_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, req.id2, ADDR_PRESENT_POSITION)
    print("Present Position of ID %s = %s" % (req.id1, dxl1_present_position))
    print("Present Position of ID %s = %s" % (req.id2, dxl2_present_position))
    res = SyncGetPositionResponse()
    res.position1 = dxl1_present_position
    res.position2 = dxl2_present_position
    return res

def pan_tilt_node():
    rospy.init_node('pan_tilt_node')
    pan_tilt_angle_topic = rospy.get_param('~pan_tilt_angle_topic', 'head_rot')
    rospy.Subscriber(pan_tilt_angle_topic, PanTiltAngle, set_goal_pos_callback)
    rospy.Service('sync_get_position', SyncGetPosition, sync_get_present_pos)
    rospy.spin()

def clamp(n, smallest, largest): return max(smallest, min(n, largest))

def angle_to_dxl_position(angle, flip = 1, multiplier = 1, offset = 0):
    return int(math.floor(dxl_zero_pos + flip * k * (multiplier * angle + offset)))

def zero_motors():
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, dxl_zero_pos)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_POSITION, angle_to_dxl_position(tilt_offset))


def main():
    # Open port
    try:
        portHandler.openPort()
        print("Succeeded to open the port")
    except:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()

    # Set port baudrate
    try:
        portHandler.setBaudRate(BAUDRATE)
        print("Succeeded to change the baudrate")
    except:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

    # Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        print("Press any key to terminate...")
        getch()
        quit()
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("Press any key to terminate...")
        getch()
        quit()
    else:
        print("DYNAMIXEL1 has been successfully connected")

    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        print("Press any key to terminate...")
        getch()
        quit()
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("Press any key to terminate...")
        getch()
        quit()
    else:
        print("DYNAMIXEL2 has been successfully connected")

    print("Zeroing Motors.")
    zero_motors()

    print("Ready to get & set Position.")

    pan_tilt_node()


if __name__ == '__main__':
    main()
