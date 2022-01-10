#!/usr/bin/env python3
import rospy
import sys
from geometry_msgs.msg import Point, Twist
from pynput.mouse import Controller
from math import pow, atan2, sqrt
from turtlesim.msg import Pose

x_size = rospy.get_param("x_size")
y_size = rospy.get_param("y_size")
d = 11

class TurtleControl:

    def __init__(self):
        self.pub = rospy.Publisher("turtle_control", Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/pose', Pose, self.update_pose)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.pose = Pose()
        self.mouse = Controller()
        self.rate = rospy.Rate(10)
        
    def update_pose(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def get_distance(self, goal_x, goal_y):
        distance = sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2))
        return d * distance

    def get_ang_distance(self, goal_x, goal_y):
        ang_distance = atan2(goal_y - self.pose.y, goal_x - self.pose.x) - self.pose.theta
        return d * ang_distance

    def run(self):
        while not rospy.is_shutdown():
            goal_pose = Pose()
            curr_position = self.mouse.position
            goal_pose.x = curr_position[0] * d / x_size
            goal_pose.y = (y_size - curr_position[1]) * d / y_size

            vel_msg = Twist()

            while self.get_distance(goal_pose.x, goal_pose.y) >= 0.01:

                vel_msg.linear.x = self.get_distance(goal_pose.x, goal_pose.y)
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0

                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = self.get_ang_distance(goal_pose.x, goal_pose.y)

                self.velocity_publisher.publish(vel_msg)
                self.rate.sleep()

            vel_msg.linear.x = vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    rospy.init_node('turtle_control', anonymous=True)
    try:
        turtle = TurtleControl()
        turtle.run()

    except rospy.ROSInterruptException:
        pass
