#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float32
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Quaternion, Twist, Pose, Point
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
import numpy as np

class FlorenceBaseController:
    # Number of seconds required to rotate 1 degree at speed of 0.1 when using base_driver.move.
    # E.g. to rotate 15 degrees to the right we'll need: self.base_driver.move(0, 0, 0.1, 15 * ROT_1_DEG_TIME)
    #ROT_1_DEG_TIME = 0.2
    
    def __init__(self):
        # This will be our node name
        rospy.init_node("florence_base_controller")

        # We will subscribe to a String command topic
        self.base_sub = rospy.Subscriber("/base_cntrl/in_cmd", String, self.on_command)
        
        # We will publish a String feedback topic
        self.base_pub = rospy.Publisher("/base_cntrl/out_result", String, queue_size=3)
        
        # We will subscribe to a Float32 command topic to rotate cw or ccw by the passed value, which will come from Unity.
        # The value is a bit arbitrary, but we'll need to make sense of it. At the moment it looks like more or less a full
        # swipe of the screen is about +-0.2, depending on whether we're going left to right or right to left. Also, smaller
        # swipes appear to be anywhere between 0.03 and 0.15- almost irrespectively of how much was intended to sweep. So
        # we will probably need to loosely rely on this number and rotate by relatively small amount even with full swipe.
        self.rotate_sub = rospy.Subscriber("/base_cntrl/rotate_x", Float32, self.on_rotate)

        # We will subscribe to a Float32 command topic to crawl forward or backwards by the passed value, which will come from Unity.
        # The value is a bit arbitrary, but we'll need to make sense of it. At the moment it looks like more or less a full
        # hand move through the screen depth is about +-0.2, depending on whether we're going far to near or near to far. Also, smaller
        # moves appear to be anywhere between 0.03 and 0.15- almost irrespectively of how much was intended to move. So
        # we will probably need to loosely rely on this number and crawl by relatively small amount even with full move.
        self.rotate_sub = rospy.Subscriber("/base_cntrl/crawl_x", Float32, self.on_crawl)
        
        # We will subscribe to an Int16 command topic to move forward by the passed amount of meters.
        self.go_fwd_sub = rospy.Subscriber("/base_cntrl/go_fwd", Float32, self.on_fwd)
        
        # We will subscribe to an Int16 command topic to move backwards by the passed amount of meters.
        self.go_back_sub = rospy.Subscriber("/base_cntrl/go_back", Float32, self.on_back)
        
        # This is how we will monitor the completion of the requested move
        self.go_back_sub = rospy.Subscriber("/odometry/filtered", Odometry, self.on_odometry_received)

        #pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Service for clearing costmaps
        self.costmap_clear = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        
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

    # This is how we'll react on the commands received
    def on_command(self, cmd):
        if cmd.data == "get_cost_of_travel":
            # publish list of path costs to all shelves
            #self.get_cost_list.publish(self.calculate_cost_of_travel())
            self.base_pub.publish("NOT IMPLEMENTED")
    
    # When user wants the robot to rotate, then this will be called with a float32 number loosely indicating
    # amount to rotate by. See comment of self.rotate_sub for more info.
    # Positive number: rotating clock wise, negative: rotating counter clock wise
    def on_rotate(self, amount):
        move = Twist()
        move.angular.z = 0.5 # constant speed of rotation
        
        cur_orientation = self.cur_odometry.pose.pose.orientation
        desired_pose = Pose(self.cur_odometry.pose.pose.position, self.rotateQuaternionAroundZ(cur_orientation, amount.data * 5)) # This will be current pose with altered orientation
        
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
    
    # When user wants the robot to go forward, then this will be called with the number of meters passed.
    def on_fwd(self, dist):
        self.base_pub.publish("OK FWD")
        
    # When user wants the robot to go forward, then this will be called with the number of meters passed.
    def on_back(self, dist):
        self.base_pub.publish("OK BACK")

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
