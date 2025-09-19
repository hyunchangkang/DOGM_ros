#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

def main():
    rospy.init_node('static_tf_publisher')
    br = tf2_ros.StaticTransformBroadcaster()

    t1 = TransformStamped()
    t1.header.stamp = rospy.Time.now()
    t1.header.frame_id = "odom"
    t1.child_frame_id = "base_link"
    t1.transform.translation.x = 0.0
    t1.transform.translation.y = 0.0
    t1.transform.translation.z = 0.0
    t1.transform.rotation.w = 1.0

    t2 = TransformStamped()
    t2.header.stamp = rospy.Time.now()
    t2.header.frame_id = "base_link"
    t2.child_frame_id = "laser_link"
    t2.transform.translation.x = 0.0
    t2.transform.translation.y = 0.0
    t2.transform.translation.z = 0.0
    t2.transform.rotation.w = 1.0

    br.sendTransform([t1, t2])
    rospy.loginfo("Static TF published: odom->base_link->laser_link")
    rospy.spin()

if __name__ == "__main__":
    main()
