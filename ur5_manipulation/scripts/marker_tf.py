#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg

import math

from tf.transformations import quaternion_from_euler


class FixedTFBroadcaster:

    def __init__(self):
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

        while not rospy.is_shutdown():
            # Run this loop at about 10Hz
            rospy.sleep(0.1)

            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = "base_link"
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = "camera_link"

            # - Translation: [0.451, -0.509, 0.603]
            # - Rotation: in Quaternion [-0.529, 0.219, 0.741, 0.350]
            #             in RPY (radian) [-0.133, 1.215, 2.166]
            #             in RPY (degree) [-7.636, 69.641, 124.104]


            t.transform.translation.x = 0.542
            t.transform.translation.y = -0.509
            t.transform.translation.z = 0.606

            # ori = quaternion_from_euler(0.0, 0.0, -3*math.pi/4)
            t.transform.rotation.x = -0.469
            t.transform.rotation.y = -0.189
            t.transform.rotation.z = -0.800
            t.transform.rotation.w = -0.322

            tfm = tf2_msgs.msg.TFMessage([t])
            self.pub_tf.publish(tfm)

if __name__ == '__main__':
    rospy.init_node('fixed_tf2_broadcaster')
    tfb = FixedTFBroadcaster()

    rospy.spin()