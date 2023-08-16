#!/usr/bin/env python
# license removed for brevity
import rospy
import math

from geometry_msgs.msg import Pose

from tf.transformations import quaternion_from_euler   

if __name__ == '__main__':
    rospy.init_node('pick_cmd', anonymous=True)
    pub = rospy.Publisher('pick_pose', Pose, queue_size=10, latch=False)
    pose = Pose()
    pose.position.x = 0.6
    pose.position.y = 0.4
    pose.position.z = 0.205
    ori = quaternion_from_euler(-math.pi, 0, -math.pi/2)
    pose.orientation.x = ori[0]
    pose.orientation.y = ori[1]
    pose.orientation.z = ori[2]
    pose.orientation.w = ori[3]
    try:
        rospy.sleep(1)
        pub.publish(pose)
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass