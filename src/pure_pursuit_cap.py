#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import numpy as np
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from math import pow, atan2, sqrt, ceil
from tf.transformations import euler_from_quaternion

x = 0.0
y = 0.0
v = 0.0
yaw = 0.0

def statesCallback(data):
    global x, y, v, yaw
    # find index of slash
    name = data.name
    index = name.index("jackal")
    #index = name.index("/")
    x = data.pose[index].position.x
    y = data.pose[index].position.y
    v = data.twist[index].linear.x
    quaternion = (
        data.pose[index].orientation.x,
        data.pose[index].orientation.y,
        data.pose[index].orientation.z,
        data.pose[index].orientation.w
    )
    euler = euler_from_quaternion(quaternion)
    yaw = euler[2]

def robotAtGoal(robx, roby, goalx, goaly):
    distance_tolerance = 0.5
    val = sqrt(pow((goalx - robx), 2) + pow((goaly - roby), 2)) 
    
    print(val)
    return val <= distance_tolerance

def getLookAheadPoint(waypoints, robx, roby, lookAheadDistance, lastIndex, lastFractionalIndex, lastLookAhead):
    for j in range(lastIndex, len(waypoints) - 1):
        E = waypoints[j]
        L = waypoints[j + 1]
        C = (robx, roby)
        r = lookAheadDistance
        d = (L[0]-E[0], L[1]-E[1])
        f = (E[0]-C[0], E[1]-C[1])
        a = np.dot(d, d)
        b = np.dot(np.multiply(2, f), d)
        c = np.dot(f, f) - r*r
        discriminant = b*b - 4*a*c
        if discriminant < 0:
            break
        discriminant = sqrt(discriminant)
        t1 = (-b - discriminant)/(2*a)
        t2 = (-b + discriminant)/(2*a)
        if 0 <= t1 <= 1 and j+t1 > lastFractionalIndex:
            return (E[0] + t1*d[0], E[1] + t1*d[1]), j, j+t1
        if 0 <= t2 <= 1 and j+t2 > lastFractionalIndex:
            return (E[0] + t2*d[0], E[1] + t2*d[1]), j, j+t2
        return lastLookAhead, lastIndex, lastFractionalIndex
        
def injectPoints(waypoints):
    spacing = 5
    new_points = []
    for j in range(0, len(waypoints)-1):
        start_point = waypoints[j]
        end_point = waypoints[j+1]
        vector = (end_point[0] - start_point[0], end_point[1] - start_point[1])
        d = sqrt(pow(vector[0], 2)+pow(vector[1], 2))
        num_points_that_fit = int(ceil(d/spacing))
        vector = (vector[0]/d * spacing, vector[1]/d * spacing)
        for i in range(0, num_points_that_fit):
            new_list = (start_point[0] + vector[0]*i, start_point[1] + vector[1]*i)
            new_points.append(new_list)
        new_points.append(end_point)
    return new_points

def smoothPath(path): # path is [(x1, y1), ..., (xend, yend)]
    b = 0.75
    a = 1-b
    tolerance = 0.001
    newPath = [list(point) for point in path] # tuples are immutable
    change = tolerance
    while change >= tolerance:
        change = 0
        for i in range(1, len(path)-1):
            for j in range(0, len(path[i])):
                aux = newPath[i][j]
                newPath[i][j] += a*(path[i][j] - newPath[i][j]) + b*(newPath[i-1][j] + newPath[i+1][j]
                                                                     - (2.0*newPath[i][j]))
                change += abs(aux-newPath[i][j])
    newPath = list(tuple(point) for point in newPath)
    return newPath

def main():
    #global x, y, v, yaw
    rospy.init_node('pure_pursuit_cap', anonymous=True)
    velocity_publisher = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=10)
    #velocity_publisher = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/gazebo/model_states', ModelStates, statesCallback)
    rate = rospy.Rate(10)
    vel_msg = Twist()
    waypoints = [(10, 0), (20, 20)]
    path = injectPoints(waypoints)
    goals_x = [10]
    goals_y = [0]
    lookAheadDistance = 9
    lastIndex = 0
    lastLookAheadIndex = 0
    lastFractionalIndex = 0
    lookAheadPoint = (0, 0)
    i = 0
    while not rospy.is_shutdown():
        if robotAtGoal(x, y, waypoints[-1][0], waypoints[-1][1]):  # switch to next goal or not
           vel_msg.linear.x = 0
           vel_msg.angular.z = 0
           velocity_publisher.publish(vel_msg)
           break 


        lookAheadPoint, lastIndex, lastFractionalIndex = getLookAheadPoint(waypoints, x, y, lookAheadDistance, lastIndex, lastFractionalIndex, lookAheadPoint)
        goal_pose_x = lookAheadPoint[0]
        goal_pose_y = lookAheadPoint[1]
        print(lookAheadPoint)
        # main_logic to compute linear_x, angular_z
        # Proportional Controller
        # linear velocity in the x-axis:
        #vel_msg.linear.x = 1.5 * sqrt(pow((goal_pose_x - x), 2) + pow((goal_pose_y - y), 2))
        vel_msg.linear.x = 2
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        # angular velocity in the z-axis:
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 2 * (atan2(goal_pose_y - y, goal_pose_x - x) - yaw)
        #print("before publish")

        # Publishing our vel_msg
        velocity_publisher.publish(vel_msg)
        print("x: %f", x)
        print("y: %f", y)
        #print("before sleep")
        rate.sleep()
        #print("before spin")
        #rospy.spin() # vehicle will not

if __name__ == "__main__":
    main()
