#!/usr/bin/env python
# license removed for brevity
import rospy
import math

from geometry_msgs.msg import Pose

from tf.transformations import euler_from_matrix, quaternion_from_euler

def pick():
    pub = rospy.Publisher('pick_pose', Pose, queue_size=10, latch=True)

    pose = Pose()

    pose.position.x = 0.5585
    pose.position.y = -0.136
    pose.position.z = 0.0155

    ori = quaternion_from_euler(0.0, 0.0, 0.0)
    pose.orientation.y = ori[1]
    pose.orientation.z = ori[2]
    pose.orientation.x = ori[0]
    pose.orientation.w = ori[3] 
    
    rospy.sleep(1)

    pub.publish(pose)

def place():
    pub = rospy.Publisher('place_pose', Pose, queue_size=10, latch=True)

    pose = Pose()

    pose.position.x = 0.136
    pose.position.y = -0.5585
    pose.position.z = 0.0155

    ori = quaternion_from_euler(0.0, 0.0, -math.pi/2)
    pose.orientation.y = ori[1]
    pose.orientation.z = ori[2]
    pose.orientation.x = ori[0]
    pose.orientation.w = ori[3] 
    
    rospy.sleep(1)

    pub.publish(pose)
    

if __name__ == '__main__':
    rospy.init_node('pick_place_pub', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    try:
        pick()
        place()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
