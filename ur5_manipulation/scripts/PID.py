#!/usr/bin/env python


from __future__ import print_function
import time
import sys
import rospy
from dynamixel_workbench_msgs.srv import *
from dynamixel_workbench_msgs.msg import *
from sensor_msgs.msg import JointState
#from dynamixel_msgs.msg import*
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32


data=[0.0 , 0.0]

def callback1(goal):
    msg = rospy.wait_for_message('/dynamixel_workbench/dynamixel_state', DynamixelStateList , timeout=5)
    current_a =msg.dynamixel_state[0].present_current
    current_b =msg.dynamixel_state[1].present_current
    vel_a=msg.dynamixel_state[0].present_velocity
    vel_b=msg.dynamixel_state[1].present_velocity
    pos_a =msg.dynamixel_state[0].present_position
    pos_b =msg.dynamixel_state[1].present_position
    pub = rospy.Publisher('/dynamixel_workbench/cmd_vel', Twist , queue_size=10)
    rospy.init_node('PID', anonymous=True)
    output = Twist()
    if current_a > 1024:
        current_a=(current_a-1024)
    else: current_a = - current_a
    
    if current_b > 1024:
        current_b=-(current_b-1024)
    if pos_a >  1010:
        vel_a=0
        #output.angular.z = 0
        #pub.publish(output)
    if 25>pos_a:
        vel_a=0
        #output.angular.z = 0
        #pub.publish(output)
    if pos_b >  1010:
        vel_b=0
        #output.angular.z = 0
        #pub.publish(output)
    if 25>pos_b:
        vel_b=0
        #output.angular.z = 0
        #pub.publish(output)
    
    Goal = int(goal.data)
    rospy.loginfo("Goal: {}".format(Goal))
    #vel=PID_control(300,pos_b,vel_b,0.008,0.0)
    vel=PID_control(Goal,current_b,vel_b,-0.005,0.0) 
    output.angular.z = vel
    pub.publish(output)
    old_vel=vel
    print("A current is : ")
    print(current_a)
    print("")
    print("B Current is : ")
    print(current_b)
    print("")
    print("A pos is : ")
    print(pos_a)
    print("")
    print("B pos is : ")
    print(pos_b)
    print("")
    
    
        

def PID_control(goal,present,old_pid,kp,kd):
    
    delta= goal-present
    out=(delta*kp+old_pid*kd)
    if goal >0:
        out = -0.75
    return out
        

def listener():

    
    #rospy.Subscriber('/dynamixel_workbench/joint_states', JointState, callback)
    rospy.Subscriber('/gripper_goal', Int32, callback1)
    #rospy.Subscriber('/dynamixel_workbench/dynamixel_state',DynamixelStateList , callback1)
    return print("test")
    

if __name__ == '__main__':
    rospy.init_node('PID', anonymous=True)
    listener()
    rospy.spin()