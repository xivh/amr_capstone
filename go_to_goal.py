#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt
import numpy as np
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion


class pure_pursuit():
    def __init__(self):
        # Creating our node,publisher and subscriber
        rospy.init_node('pure_pursuit_cap', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=1)
        self.states_subscriber = rospy.Subscriber('/odometry/filtered', Odometry, self.statesCallback)
        self.rate = rospy.Rate(10)

    # Callback function implementing the pose value received
    def statesCallback(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.v = data.twist.twist.linear.x
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        )
        euler = euler_from_quaternion(quaternion)
        self.yaw = euler[2]
        # print("curr_x: %f", self.x)
        # print("curr_y: %f", self.y)
        # rospy.loginfo("Angular Velocity = %f", data.angular_velocity.z)

    def get_distance(self, goal_x, goal_y):
        distance = sqrt(pow((goal_x - self.x), 2) + pow((goal_y - self.y), 2))
        return distance

    def move2goal(self, goal_pose_x, goal_pose_y):
        distance_tolerance = 0.5
        vel_msg = Twist()
        print("goal_x: %f", goal_pose_x)
        print("goal_y: %f", goal_pose_y)
        print("curr_x: %f", self.x)
        print("curr_y: %f", self.y)
        while sqrt(pow((goal_pose_x - self.x), 2) + pow((goal_pose_y - self.y), 2)) >= distance_tolerance:
            # Porportional Controller
            # linear velocity in the x-axis:
            vel_msg.linear.x = 1.5 * sqrt(pow((goal_pose_x - self.x), 2) + pow((goal_pose_y - self.y), 2))
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 4 * (atan2(goal_pose_y - self.y, goal_pose_x - self.x) - self.yaw)
            print("before publish")
            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            print("angular: %f", vel_msg.angular.z)
            print("linear: %f", vel_msg.linear.x)
            print("before sleep")
            self.rate.sleep()
            print("before spin")
            rospy.spin()
            print("after spin")
        # Stopping our robot after the movement is over
        print("out of loop")
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        rospy.spin()


if __name__ == '__main__':
    try:
        # Testing our function
        x = pure_pursuit()
        rospy.spin()
        goals_x = [10]
        goals_y = [0]
        i = 0
        while i < len(goals_x):
            gx = goals_x[i]
            gy = goals_y[i]
            x.move2goal(10, 0)
            i += 1

    except rospy.ROSInterruptException: pass