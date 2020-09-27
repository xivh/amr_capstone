#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def statesCallback(data):
    ang_vel = data.twist.twist.angular.z
    print(ang_vel)

def main():
    vel_pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=1)
    rospy.init_node('jackal_turn', anonymous=True)
    rospy.Subscriber('/odometry/filtered', Odometry, statesCallback)
    rate = rospy.Rate(10)
    vel = Twist()
    vel.linear.x = 0.5
    vel.angular.z = 1.0
    count = 0
    while not rospy.is_shutdown():
        vel_pub.publish(vel)
        rate.sleep()
        count+=1
        if(count==200):
            vel.linear.x = 0.0
            vel.angular.z = 0.0
            break

if __name__ == "__main__":
    main()
