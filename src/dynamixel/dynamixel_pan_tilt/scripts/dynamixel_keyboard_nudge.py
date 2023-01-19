#!/usr/bin/env python

import os
import rospy
from dynamixel_sdk import *
from dynamixel_sdk_examples.srv import *
from dynamixel_sdk_examples.msg import *

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, select, tty, termios
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
DXL1_ID                     = 11                 # Dynamixel ID : 1
DXL2_ID	                    = 12
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

def set_sync_goal_pos_callback(data):
    # print("Set Goal Position of ID %s = %s" % (data.id1, data.position1))
    # print("Set Goal Position of ID %s = %s" % (data.id2, data.position2))
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, data.id1, ADDR_GOAL_POSITION, data.position1)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, data.id2, ADDR_GOAL_POSITION, data.position2)

def set_sync_goal_position(dxl1_pos, dxl2_pos):
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, dxl1_pos)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_POSITION, dxl2_pos)

def sync_get_present_pos(req):
    dxl1_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, req.id1, ADDR_PRESENT_POSITION)
    dxl2_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, req.id2, ADDR_PRESENT_POSITION)
    print("Present Position of ID %s = %s" % (req.id1, dxl1_present_position))
    print("Present Position of ID %s = %s" % (req.id2, dxl2_present_position))
    res = SyncGetPositionResponse()
    res.position1 = dxl1_present_position
    res.position2 = dxl2_present_position
    return res

def sync_get_present_pos():
    dxl1_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRESENT_POSITION)
    dxl2_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL2_ID, ADDR_PRESENT_POSITION)
    return dxl1_present_position, dxl2_present_position

def sync_read_write_py_node():
    rospy.init_node('sync_read_write_py_node')
    rospy.Subscriber('sync_set_position', SyncSetPosition, set_sync_goal_pos_callback)
    rospy.Service('sync_get_position', SyncGetPosition, sync_get_present_pos)
    rospy.spin()

def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

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

    print("Ready to get & set Position.")


if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)

    main()

    shift = 10

    try:
        while True:
            key = getKey(None)
            if key == 'w':
                dxl1_pos, dxl2_pos = sync_get_present_pos()
                set_sync_goal_position(dxl1_pos, dxl2_pos - shift)

            if key == 'a':
                dxl1_pos, dxl2_pos = sync_get_present_pos()
                set_sync_goal_position(dxl1_pos + shift, dxl2_pos)

            if key == 's':
                dxl1_pos, dxl2_pos = sync_get_present_pos()
                set_sync_goal_position(dxl1_pos, dxl2_pos + shift)

            if key == 'd':
                dxl1_pos, dxl2_pos = sync_get_present_pos()
                set_sync_goal_position(dxl1_pos - shift, dxl2_pos)

    except KeyboardInterrupt:
        print('Interrupted!')
