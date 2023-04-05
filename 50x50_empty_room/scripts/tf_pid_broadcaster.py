#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
from geometry_msgs.msg import TransformStamped


def tf_static_broadcaster(waypoint):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "/map"
    t.child_frame_id = "/pid_frame"
    t.transform.translation.x = waypoint[0]
    t.transform.translation.y = waypoint[1]
    t.transform.translation.z = waypoint[2]
    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 1.0

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('tf_pid_broadcaster')
    waypoint = [5.0,5.0,3.0]

    rate = rospy.Rate(1.0)

    while not rospy.is_shutdown():

        tf_static_broadcaster(waypoint)

        rate.sleep()