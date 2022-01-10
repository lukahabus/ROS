#!/usr/bin/env python3
import numpy as np
import rospy
from geometry_msgs.msg import Point
import math

# In this example, we are using `visualization_msgs.Marker`, which has
# an array of `geometry_msgs.Point` member named `points`.
from visualization_msgs.msg import Marker


def create_publish_marker(pub, initial_time):
    now = rospy.Time.now()

    # `now` and `begin` are rospy.Time objects. Their difference is a
    # rospy.Duration, which can be converted into a float seconds using
    # the `to_sec()` method.
    # See:
    # http://wiki.ros.org/rospy/Overview/Time
    t = (now - initial_time).to_sec()

    marker = Marker()
    # To be able to display the points in RViz, we have to fill out
    # the color, opacity, point size, and the reference frame.
    # Read http://wiki.ros.org/rviz/DisplayTypes/Marker#Points_.28POINTS.3D8.29
    # for more information.
    marker.header.frame_id = "ellipse"
    marker.header.stamp = now
    # These values will affect how the Marker message is visualized in RViz.
    marker.type = Marker.POINTS
    # Alpha value: 0 = invisible (fully transparent), 1 = visible (opaque)
    marker.color.a = 1.
    # Red, green and blue
    marker.color.r, marker.color.g, marker.color.b = (1., 0., 0.)
    # Width and height
    marker.scale.x = .5
    marker.scale.y = .5

    # Visualization example: an ellipse whose major axis is oscillating with time,
    # parameterized by `u` with 30 samples.
    u = np.linspace(0, 2 * math.pi, 30)

    r1 = 4 + 3 * math.sin(t)
    r2 = 4 + 4 * math.cos(t)

    x = r1 * np.sin(u)
    y = r2 * np.cos(u)

    for xy in zip(x, y):
        p = Point()
        p.x, p.y = xy
        marker.points.append(p)

    pub.publish(marker)


if __name__ == "__main__":
    rospy.init_node('ellipse_points')
    pub = rospy.Publisher('point_positions', Marker, queue_size=1)

    # Used for animating the ellipse
    initial_time = rospy.Time.now()

    # Timer which calls the callback for calculating and publishing ellipse points
    # with a frequency of 25 Hz
    rospy.Timer(rospy.Duration(1 / 25), 
               lambda unused_event: create_publish_marker(pub, initial_time))

    rospy.spin()
