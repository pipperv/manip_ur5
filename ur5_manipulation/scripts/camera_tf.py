#!/usr/bin/env python

import rospy
import math

import tf2_ros
from tf2_geometry_msgs import do_transform_pose

from tf.transformations import quaternion_from_euler, quaternion_multiply

from geometry_msgs.msg import TransformStamped, Quaternion, Vector3, PoseStamped

def invert_pose_stamped(pose_stamped):
    inverted_pose_stamped = PoseStamped()
    inverted_pose_stamped.header = pose_stamped.header

    # Invert the translation
    inverted_pose_stamped.pose.position.x = -pose_stamped.pose.position.x
    inverted_pose_stamped.pose.position.y = -pose_stamped.pose.position.y
    inverted_pose_stamped.pose.position.z = -pose_stamped.pose.position.z

    # Invert the orientation
    inverted_pose_stamped.pose.orientation.x = -pose_stamped.pose.orientation.x
    inverted_pose_stamped.pose.orientation.y = -pose_stamped.pose.orientation.y
    inverted_pose_stamped.pose.orientation.z = -pose_stamped.pose.orientation.z
    inverted_pose_stamped.pose.orientation.w = pose_stamped.pose.orientation.w

    return inverted_pose_stamped


def quaternion_trans(q1, q2):
    q3 = Quaternion()
    v1 = [q1.x,q1.y,q1.z,q1.w]
    v2 = [q2.x,q2.y,q2.z,q2.w]
    v3 = quaternion_multiply(v1,v2)
    q3.x = v3[0]
    q3.y = v3[1]
    q3.z = v3[2]
    q3.w = v3[3]
    # q3.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
    # q3.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y
    # q3.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x
    # q3.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
    return q3

def translation_add(v1, v2):
    result = Vector3()
    result.x = v1.x + v2.x
    result.y = v1.y + v2.y
    result.z = v1.z + v2.z
    return result

def pose_stamped_to_transform_stamped(pose_stamped):
    # Create a TransformStamped object from the PoseStamped
    transform_stamped = TransformStamped()
    transform_stamped.header = pose_stamped.header
    transform_stamped.child_frame_id = "camera_color_optical_frame"
    transform_stamped.transform.translation = pose_stamped.pose.position
    transform_stamped.transform.rotation = pose_stamped.pose.orientation

    return transform_stamped

def aruco_transform_callback(data):

    tf_broadcaster = tf2_ros.TransformBroadcaster()
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    tf_buffer.set_transform(data, "aruco_tf")
    inverse_tf = tf_buffer.lookup_transform("marker_frame","camera_link", rospy.Time(),rospy.Duration(1.0))

    inverse_tf.header.frame_id = "aruco_frame"
    inverse_tf.child_frame_id = "camera_link"
    # inverse_tf.header.stamp = rospy.Time.now()
    
    tf_broadcaster.sendTransform(inverse_tf)
    
    

def main():
    rospy.init_node('aruco_transform_to_tf_broadcaster')

    # Create a TF broadcaster
    

    # Subscribe to the /aruco_single/transform topic
    rospy.Subscriber("/aruco_single/transform", TransformStamped, aruco_transform_callback)

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
