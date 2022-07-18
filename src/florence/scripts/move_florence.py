#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float32, Empty
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Quaternion, Twist, Pose, Point
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty as EmptySrv
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
import numpy as np
import threading
from enum import Enum

Crawl_Thread_State = Enum('Crawl_Thread_State', 'idle crawling have_to_stop stopping')
Rotate_Thread_State = Enum('Rotate_Thread_State', 'idle turning have_to_stop stopping')

class FlorenceBaseController:
    # Number of seconds required to rotate 1 degree at speed of 0.1 when using base_driver.move.
    # E.g. to rotate 15 degrees to the right we'll need: self.base_driver.move(0, 0, 0.1, 15 * ROT_1_DEG_TIME)
    #ROT_1_DEG_TIME = 0.2
    
    def __init__(self):
        # This will be our node name
        rospy.init_node("florence_base_controller")

        # Crawling thread initial state
        self.crawl_state = Crawl_Thread_State.idle
        self.crawl_thread = None
        
        # Rotation thread initial state
        self.rotation_state = Rotate_Thread_State.idle
        self.rotation_thread = None
        
        # We will publish a String feedback topic
        self.base_pub = rospy.Publisher("/base_cntrl/out_result", String, queue_size=3)
        
        # We will subscribe to an emergency stop command topic. No data needed here. We just need to stop the robot when
        # this is called.
        self.base_sub = rospy.Subscriber("/base_cntrl/stop", Empty, self.on_stop)
        
        # We will subscribe to a Float32 command topic to rotate cw or ccw by the passed value, which will come from Unity.
        # The value is a bit arbitrary, but we'll need to make sense of it. At the moment it looks like more or less a full
        # swipe of the screen is about +-0.2, depending on whether we're going left to right or right to left. Also, smaller
        # swipes appear to be anywhere between 0.03 and 0.15- almost irrespectively of how much was intended to sweep. So
        # we will probably need to loosely rely on this number and rotate by relatively small amount even with full swipe.
        self.rotate_sub = rospy.Subscriber("/base_cntrl/rotate_x", Float32, self.on_rotate2)

        # We will subscribe to a Float32 command topic to crawl forward or backwards by the passed value, which will come from Unity.
        # The value is a bit arbitrary, but we'll need to make sense of it. At the moment it looks like more or less a full
        # hand move through the screen depth is about +-0.2, depending on whether we're going far to near or near to far. Also, smaller
        # moves appear to be anywhere between 0.03 and 0.15- almost irrespectively of how much was intended to move. So
        # we will probably need to loosely rely on this number and crawl by relatively small amount even with full move.
        self.rotate_sub = rospy.Subscriber("/base_cntrl/crawl_x", Float32, self.on_crawl2)

        # Another way to control the robot, is continuous- the offset from start point is the speed of either rotation or movement in the Z direction.
        self.rotate_sub = rospy.Subscriber("/base_cntrl/continuous", Twist, self.on_drive_continuously)
        
        # This is how we will monitor the completion of the requested move
        self.go_back_sub = rospy.Subscriber("/odometry/filtered", Odometry, self.on_odometry_received)

        #pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Service for clearing costmaps
        self.costmap_clear = rospy.ServiceProxy('/move_base/clear_costmaps', EmptySrv)
        
        # Subscribe to the move_base action server. This lets us to directly interact with move_base- to ask for paths and set goals.
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # Clean up on shutdown        
        rospy.on_shutdown(self.shutdown)

        rate = rospy.Rate(10) # 10hz
        
        # Global variables for keeping track
        # Current odometry
        self.cur_odometry = Odometry()
        
        # Goal ID
        self.goal_id = 0
        
        # Lock for our rotations and crawls
        self._lock = threading.Lock()

        # Last Twist command sent to the continuous drive topic
        self.last_cont_drv_msg = Twist()

    # Creates an MoveBaseGoal object from a Pose and moves to it
    def move_to_pose(self, pose, report=True):
    
        # Because the point on the robot, which we're trying to get to the target pose, is offset by an angle,
        # we need to rotate the given pose by around pi/2 counter clock wise.
        #pose = Pose(pose.position, self.rotateQuaternionAroundZ(pose.orientation, -pi/2))
        
        # Intialize the goal
        goal = MoveBaseGoal()
        
        # Use the map frame to define goal poses
        goal.target_pose.header.frame_id = 'odom'
        
        # Set the goal ID
        self.goal_id += 1
        goal.target_pose.header.seq = self.goal_id
        
        # Set the time stamp to "now"
        goal.target_pose.header.stamp = rospy.Time.now()

        #print(pose)

        # Set the goal pose to the shelf
        goal.target_pose.pose = pose
        
        # Start the robot moving toward the goal
        return self.move(goal, report)

    # Moves the base to the passed goal, which includes a Pose
    # report - a flag of whether we want the function to publish results of the move to self.base_pub
    # Returns a boolean flag- True when the move succeeded, False if not 
    def move(self, goal, report=True):

        # First clear maps if we have any artefacts from previous motions
        resp = self.costmap_clear()
        #self.space_clear(Empty())
        
        # Send the goal pose to the MoveBaseAction server
        self.move_base.send_goal(goal)
        
        # Allow 45 seconds to get there
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(45)) 
        
        # If we don't get there in time, abort the goal
        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
            self.base_pub.publish("TIMEOUT")
            return False
        else:
            # We made it!
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                if report:
                    rospy.loginfo("Goal succeeded!")
                    self.base_pub.publish("OK MOVE")
                return True
            else:
                if report:
                    self.base_pub.publish("BAD MOVE")
                return False
    
    # When user wants the robot to rotate, then this will be called with a float32 number loosely indicating
    # amount to rotate by. See comment of self.rotate_sub for more info.
    # Positive number: rotating clock wise, negative: rotating counter clock wise
    def on_rotate(self, amount):
        move = Twist()
        move.angular.z = 0.5 # constant speed of rotation
        
        cur_orientation = self.cur_odometry.pose.pose.orientation
        desired_pose = Pose(self.cur_odometry.pose.pose.position, self.rotateQuaternionAroundZ(cur_orientation, amount.data * -5)) # This will be current pose with altered orientation
        
        self.move_to_pose(desired_pose, False)
        #self.base_pub.publish("OK ROTATE")
    
    # When user wants the robot to move forward, then this will be called with a float32 number loosely indicating
    # amount to move forward by (or backwards if the number is negative). See comment of self.crawl_sub for more info.
    # Positive number: moving forward, negative: moving backwards
    def on_crawl(self, amount): 
        cur_position = self.cur_odometry.pose.pose.position
        desired_pose = Pose(Point(cur_position.x + amount.data * 50, cur_position.y, cur_position.z), self.cur_odometry.pose.pose.orientation) # This will be current pose with altered orientation
        
        self.move_to_pose(desired_pose, False)
        #self.base_pub.publish("OK ROTATE")
    
    # This is how we will monitor the completion of the requested move 
    def on_odometry_received(self, odometry):
        self.cur_odometry = odometry
    
    # This is what our crawl thread will be executing    
    def crawler_thread_function(self, amount):
        self.crawl_state = Crawl_Thread_State.idle

        cmd = Twist()
        if (amount.data > 0):
            cmd.linear.x = 0.5 # constant speed of crawl forward
        else:
            cmd.linear.x = -0.5 # constant speed of crawl backwards

        cmd.linear.y = 0
        cmd.angular.z = 0
        rospy.loginfo('Sending dX={}, dY={}, dT={} for {}s'.format(0.5,
                                                                   0,
                                                                   0,
                                                                   amount.data * 50))
        duration = round(abs(amount.data) * 50)
        r = rospy.Rate(10)

        # Check one last time that we're good to go
        with self._lock:
            if (self.crawl_state == Crawl_Thread_State.idle):
                self.crawl_state = Crawl_Thread_State.crawling
        
        for ii in range(int(10*duration)):
            r.sleep()
            # check if we need to stop
            if (self.crawl_state == Crawl_Thread_State.have_to_stop):
                break
            # otherwise send the Twist message to the cmd_vel topic
            self.cmd_vel_pub.publish(cmd)
        #rospy.loginfo('Stopping')
        self.crawl_state = Crawl_Thread_State.stopping
        cmd = Twist()
        for ii in range(10):
            r.sleep()
            self.cmd_vel_pub.publish(cmd)
        
        # finally mark the thread as done
        self.crawl_state = Crawl_Thread_State.idle
            
    # When user wants the robot to go forward, then this will be called with the number of meters passed.
    def on_crawl2(self, amount):
        # First if we already have some thread running, then tell it to stop and wait for it.
        if (self.crawl_state != Crawl_Thread_State.idle):
            self.crawl_state = Crawl_Thread_State.have_to_stop
            self.crawl_thread.join()
    
        # Now create a brand new thread and let it run
        self.crawl_thread = threading.Thread(target=self.crawler_thread_function, args=(amount,), daemon=True)
        
        self.crawl_thread.start()

    # This needs to take in a Twist message, analyse it for sanity, perhaps filter it and then pass it to cmd_vel
    def on_drive_continuously(self, twist_msg):

        if (abs(twist_msg.linear.x) < 0.1 or abs(twist_msg.linear.x) > 0.5):
            twist_msg.linear.x = 0

        if (abs(twist_msg.angular.z) < 0.1 or abs(twist_msg.angular.z) > 0.5):
            twist_msg.angular.z = 0

        self.cmd_vel_pub.publish(twist_msg)

        # Remember the last twist message that was sent to cmd_vel
        self.last_cont_drv_msg = twist_msg

    # This is what our crawl thread will be executing
    def rotation_thread_function(self, amount):
        self.rotation_state = Rotate_Thread_State.idle

        cmd = Twist()
        if (amount.data > 0):
            cmd.angular.z = 0.5 # constant speed of crawl forward
        else:
            cmd.angular.z = -0.5 # constant speed of crawl backwards

        duration = round(abs(amount.data) * 5)
        r = rospy.Rate(10)
        
        # Check one last time that we're good to go
        with self._lock:
            if (self.rotation_state == Rotate_Thread_State.idle):
                self.rotation_state = Rotate_Thread_State.turning
        
        for ii in range(int(10*duration)):
            r.sleep()
            # check if we need to stop
            if (self.rotation_state == Rotate_Thread_State.have_to_stop):
                break
            # otherwise send the Twist message to the cmd_vel topic
            self.cmd_vel_pub.publish(cmd)
        #rospy.loginfo('Stopping')
        self.rotation_state = Rotate_Thread_State.stopping
        cmd = Twist()
        for ii in range(10):
            r.sleep()
            self.cmd_vel_pub.publish(cmd)
        
        # finally mark the thread as done
        self.rotation_state = Rotate_Thread_State.idle

    # When user wants the robot to rotate, then this will be called with a float32 number loosely indicating
    # amount to rotate by. See comment of self.rotate_sub for more info.
    # Positive number: rotating clock wise, negative: rotating counter clock wise
    def on_rotate2(self, amount):
        # First if we already have some thread running, then tell it to stop and wait for it.
        if (self.rotation_state != Rotate_Thread_State.idle):
            self.rotation_state = Rotate_Thread_State.have_to_stop
            self.rotation_thread.join()
    
        # Now create a brand new thread and let it run
        self.rotation_thread = threading.Thread(target=self.rotation_thread_function, args=(amount,), daemon=True)
        
        self.rotation_thread.start()
        
    # When this is called, all we want to do is stop fully.
    def on_stop(self):
        # Make sure that we're not in the middle of changing the rotation or crawl states
        with self._lock:
            # Stop any rotation that is going on.
            if (self.rotation_state != Rotate_Thread_State.idle):
                self.rotation_state = Rotate_Thread_State.have_to_stop

            # Stop any crawling that is going on.
            if (self.crawl_state != Crawl_Thread_State.idle):
                self.crawl_state = Crawl_Thread_State.have_to_stop
            
        # Cancel any goal that path planning is trying to achieve
        self.move_base.cancel_goal()
    
    def shutdown(self):
        rospy.loginfo("Stopping Florence base controller...")
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

    # Conversion of Euler angle to a quaternion
    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return Quaternion(qx, qy, qz, qw)

    # Multiplication of quaternions
    def multQ(self, Q1, Q2):
        x1 = Q1.x
        y1 = Q1.y
        z1 = Q1.z
        w1 = Q1.w
        
        x2 = Q2.x
        y2 = Q2.y
        z2 = Q2.z
        w2 = Q2.w
        
        # Some quaternion multiplication maths as per Wikipedia.
        # a = w
        # b = x
        # c = y
        # d = z
        return Quaternion(
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2
        )

    # Rotates a given Quaternion around Z axis by the given amount of radians- a.k.a multiplies the source quaternion with the rotation quaternion.
    def rotateQuaternionAroundZ(self, quat, rotation):
        #quat_star = Quaternion(-quat.x, -quat.y, -quat.z, quat.w)
        quat_rotation = self.euler_to_quaternion(0, 0, rotation)
        
        #result = self.multQ(self.multQ(quat_rotation, quat), quat_star)
        result = self.multQ(quat, quat_rotation)
        return result

if __name__ == '__main__':
    try:
        FlorenceBaseController()
        while not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Florence Base Controller terminating.")
