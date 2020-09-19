import rospy
from geometry_msgs.msg  import Twist
from math import pow,atan2,sqrt
import numpy as np
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion


class pure_pursuit():
    def __init__(self):
        # Creating our node,publisher and subscriber
        # rospy.init_node('turtlebot_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
        self.states_subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, self.statesCallback)
        self.rate = rospy.Rate(10)

    # Callback function implementing the pose value received
    def statesCallback(self, data):
        self.x = data.pose.position.x
        self.y = data.pose.position.y
        self.v = data.twist.linear.x
        quaternion = (
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.yaw = euler[2]
        # rospy.loginfo("Angular Velocity = %f", data.angular_velocity.z)

    def get_distance(self, goal_x, goal_y):
        distance = sqrt(pow((goal_x - self.x), 2) + pow((goal_y - self.y), 2))
        return distance

    def move2goal(self, goal_pose_x, goal_pose_y):
        distance_tolerance = 0.1
        vel_msg = Twist()

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

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        # Stopping our robot after the movement is over
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        rospy.spin()


if __name__ == '__main__':
    try:
        # Testing our function
        x = pure_pursuit()
        rospy.spin()
        goals_x = []
        goals_y = []
        i = 0
        while i < len(target_x):
            gx = goals_x[i]
            gy = goals_y[i]
            x.move2goal(gx, gy)
            i += 1

    except rospy.ROSInterruptException: pass