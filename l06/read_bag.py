#! /usr/bin/env python3

""" A script for reading rosbag data. """
import rosbag
import rospy
from math import pow, atan2, sqrt

if __name__ == '__main__':
    bag = rosbag.Bag('turtlefollow_2021-12-13-15-04-46.bag')
    last_x = last_y = distance = 0
    for (topic, msg, t) in bag.read_messages(topics=['/turtle1/pose']):
        if last_x == 0:
            initial_time = t
        time = (t - initial_time).to_sec()
        distance += sqrt(pow((last_x - msg.x), 2) + pow((last_y - msg.y), 2))
        last_x = msg.x
        last_y = msg.y
    print(f'  Covered distance: {distance:.2f} m')
    print(f'  Average velocity: {(distance / time):.2f} m/s')
    print(f'Follow session duration: {time:.2f} s')

    