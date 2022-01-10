#!/usr/bin/env python3
import rosbag
import sys

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(f'Usage: {sys.argv[0]} input.bag output.bag')
        sys.exit()

    inbag_filename = sys.argv[1]
    outbag_filename = sys.argv[2]

    print(f'Processing input bagfile: {inbag_filename}')
    msg_counter = 0

    with rosbag.Bag(outbag_filename, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(inbag_filename, 'r').read_messages():
            # We have an opportunity to write some code for handling a message.
            # See http://wiki.ros.org/rosbag/Cookbook for more examples!``
            outbag.write(topic, msg, t)
            msg_counter += 1

    print(f'Wrote {msg_counter} messages to {outbag_filename}.')
