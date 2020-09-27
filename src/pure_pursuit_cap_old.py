#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt
from tf.transformations import euler_from_quaternion

x = 0.0
y = 0.0
v = 0.0
yaw = 0.0

def statesCallback(data):
    global x, y, v, yaw
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    v = data.twist.twist.linear.x
    quaternion = (
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w
    )
    euler = euler_from_quaternion(quaternion)
    yaw = euler[2]


def robotAtGoal(robx, roby, goalx, goaly):
    distance_tolerance = 0.25
    return sqrt(pow((goalx - robx), 2) + pow((goaly - roby), 2)) <= distance_tolerance


def main():
 																																																																																																																												
    global x, y, v, yaw
    rospy.init_node('pure_pursuit_cap', anonymous=True)
    velocity_publisher = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/odometry/filtered', Odometry, statesCallback)
    rate = rospy.Rate(10)
    vel_msg = Twist()
    goals_x = [10, 0]
    goals_y = [15, 0]
    
    i = 0
    goal_pose_x = goals_x[i]
    goal_pose_y = goals_y[i]
    # rospy.spin()
    print("cur x %f:", x)
    print("cur y %f:", y)

    while not rospy.is_shutdown():
        if robotAtGoal(x, y, goal_pose_x, goal_pose_y):  # switch to next goal or not
            i += 1
            goal_pose_x = goals_x[i]
            goal_pose_y = goals_y[i]
           
        print("cur x %f:", x)
        print("cur y %f:", y)
        # main_logic to compute linear_x, angular_z
        # Proportional Controller
        # linear velocity in the x-axis:
        vel_msg.linear.x = 1.5 * sqrt(pow((goal_pose_x - x), 2) + pow((goal_pose_y - y), 2))
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        # angular velocity in the z-axis:
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 4 * (atan2(goal_pose_y - y, goal_pose_x - x) - yaw)
        
        # Publishing our vel_msg
        velocity_publisher.publish(vel_msg)
        rate.sleep()
        # rospy.spin()


if __name__ == "__main__":
    main()
