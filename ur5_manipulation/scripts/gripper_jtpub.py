#!/usr/bin/env python
# license removed for brevity
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from dynamixel_workbench_msgs.msg import DynamixelStateList

### CONST
_OPEN = [1.0, -1.0]
_CLOSED = [0.0, -0.0]
_FULLY_OPEN = [1.3, -1.3]
_TEST = [0.46, -0.46]

def detect_width():
    grip(_CLOSED)
    msg = rospy.wait_for_message('/dynamixel_workbench/joint_states',JointState,timeout=5)
    print(msg)
    init_eff_a =msg.effort[0]
    init_eff_b =msg.effort[1]
    TH = 45
    width = []
    while len(width) == 0:
        msg = rospy.wait_for_message('/dynamixel_workbench/joint_states',JointState,timeout=5)
        eff_a =abs(msg.effort[0])
        eff_b =abs(msg.effort[1])
        dif_a = abs(eff_a-init_eff_a)
        dif_b = abs(eff_b-init_eff_b)
        if (eff_a > TH) and (eff_b > TH):
            msg_width = msg.position
            width.append(msg_width[0]-0.01)
            width.append(msg_width[1]+0.01)
            print(width)
            grip(width)

    

def grip(pos=_CLOSED):
    pub = rospy.Publisher('gripper_controller/command', JointTrajectory, queue_size=10)
    rospy.init_node('gripper_goal', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    goal = "closed"
    jt = JointTrajectory()
    jt.joint_names = ["joint1","joint2"]

    jtp = JointTrajectoryPoint()
    jtp.time_from_start.secs = 3
    jtp.positions = pos
    
    rospy.sleep(1)

    jt.points = [jtp]
    pub.publish(jt)

if __name__ == '__main__':
    try:
        grip(_OPEN)
        rospy.sleep(1)
        grip(_TEST)
        rospy.sleep(1)
    except rospy.ROSInterruptException:
        pass
