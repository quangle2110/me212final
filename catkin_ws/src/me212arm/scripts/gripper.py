#!/usr/bin/python

# 2.12 running the gripper for bottle opening
# Nastasia Winey Dec 2017
import rospy
import std_msgs.msg, sensor_msgs.msg
import numpy as np


rospy.init_node("run_gripper")

def gripper():
    exec_joint1_torque_limit_pub = rospy.Publisher('/joint1_torque_controller/set_torque_limit', std_msgs.msg.Float64, queue_size=1)

    exec_joint1_torque_enable_pub = rospy.Publisher('/joint1_torque_controller/torque_enable', std_msgs.msg.Bool, queue_size=1)     
    
    exec_joint1_torque_vel_pub = rospy.Publisher('/joint1_controller/command', std_msgs.msg.Float64, queue_size=1)

#use_real_arm = rospy.get_param('/real_arm', False)

if __name__=="__main__":
    
#    robotjoints = rospy.wait_for_message('/joint_states', sensor_msgs.msg.JointState)
#    q0 = robotjoints.position
need_to_grip = 0  
target_torque = 0.5
while need_to_grip ==1
    exec_joint1_torque_limit_pub.publish(std_msg.Float64(target_torque))
    exec_joint1_torque_enable_pub.publish(std_msg.Bool(1))
else
    exec_joint1_torque_limit_pub.publish(std_msg.Float64(0))
    exec_joint1_torque_enable_pub.publish(std_msg.Bool(0))



