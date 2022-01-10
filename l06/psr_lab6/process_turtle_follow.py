#!/usr/bin/env python3
import rosbag
import sys
from math import pow, atan2, sqrt

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f'Usage: {sys.argv[0]} input.bag')
        sys.exit()

    inbag_filename = sys.argv[1]
    outbag_filename = 'processed_follow.bag'

    print(f'Processing input bagfile: {inbag_filename}')
    msg_counter = 0
    print('Follower turtle')

    bag = rosbag.Bag(inbag_filename)
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

    with rosbag.Bag(outbag_filename, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(inbag_filename, 'r').read_messages():
            if(topic == '/turtle1/pose'):
                outbag.write('/follower/pose', msg, t)
                msg_counter += 1
            elif(topic == '/mouse_position'):
                msg.x = round(msg.x * 800 / 1920)
                msg.y = round(msg.y * 600 / 1080)
                outbag.write('/mouse_positions_on_grandparents_computer', msg, t)
                msg_counter += 1
    print(f'Wrote {msg_counter} messages to {outbag_filename}.')
