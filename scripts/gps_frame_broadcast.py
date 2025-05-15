#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

if __name__ == '__main__':
    rospy.init_node('gps_tf_broadcaster')

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "gps_utm"
    static_transformStamped.child_frame_id = "odom_utm"

    static_transformStamped.transform.translation.x = 0.0
    static_transformStamped.transform.translation.y = 0.0
    static_transformStamped.transform.translation.z = 0.0

    static_transformStamped.transform.rotation.x = 0.0
    static_transformStamped.transform.rotation.y = 0.0
    static_transformStamped.transform.rotation.z = 0.0
    static_transformStamped.transform.rotation.w = 1.0

    broadcaster.sendTransform(static_transformStamped)
    rospy.spin() 