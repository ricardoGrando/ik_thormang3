#!/usr/bin/env python


import rospy
from ik_thormang3.msg import JointPosition,JacobianMatrix
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import numpy as np
from utils import *
from ik_jacobian import *


class LinearPlanner:

    def __init__(self):
        self.joint_pose = []
        
        self.left_arm_position = []
        self.right_arm_position = []
        
        self.IK_J = IK_Jacobian()
        
    
   
    
    def linearplanner_callback(self,msg):
        print("px: " + str(msg.px) + " py: " + str(msg.py) + " pz: " + str(msg.pz))
    	print("ow: " + str(msg.ow) + " ox: " + str(msg.ox) + " oy: " + str(msg.oy) + " oz: " + str(msg.oz))
    	self.target_pose = np.array([msg.px, msg.py, msg.pz, msg.ow, msg.ox, msg.oy, msg.oz])
    	
    	arr = []
    	num = 100
    	arr.append(np.linspace(self.left_arm_position[0],self.target_pose[0],num))
    	arr.append(np.linspace(self.left_arm_position[1],self.target_pose[1],num))
    	arr.append(np.linspace(self.left_arm_position[2],self.target_pose[2],num))
    	arr.append(np.linspace(self.left_arm_position[3],self.target_pose[3],num))
    	arr.append(np.linspace(self.left_arm_position[4],self.target_pose[4],num))
    	arr.append(np.linspace(self.left_arm_position[5],self.target_pose[5],num))
    	arr.append(np.linspace(self.left_arm_position[6],self.target_pose[6],num))
    	
    		
    	arr = np.array(arr).T
    	
    	for j in range(1,num):
    	    
    	    curr = self.left_arm_position.copy()
    	    delta = cal_rpy_delta(arr[j] , curr)
    	    result = self.IK_J.cal(self.joint_pose, delta, "left_arm")
    	    joint = JointState()
            joint.name = result.keys()
            joint.position = result.values()
            self.set_joint_pub.publish(joint)

            
    def present_joint_states_callback(self,msg):
        self.joint_pose = dict(zip(msg.name, msg.position))
        self.fk_pub.publish(msg)
        
    def left_arm_position_callback(self,msg):
        self.left_arm_position = np.array([msg.px, msg.py, msg.pz, msg.ow, msg.ox, msg.oy, msg.oz])
    	
    def right_arm_position_callback(self,msg):
        self.right_arm_position = [msg.px, msg.py, msg.pz, msg.ow, msg.ox, msg.oy, msg.oz]
        
    def run(self):
        print("run LinearPlanner")
        rospy.init_node('linearplanner', anonymous=True)
        self.rate = rospy.Rate(60)
        
        ## Publisher
        self.set_joint_pub = rospy.Publisher('/robotis/set_joint_states', JointState, queue_size=10)
        self.fk_pub = rospy.Publisher('/thormang3/fk_set_joint_states', JointState, queue_size=10)
        #self.cal_jacobian_pub = rospy.Publisher('/thormang3/cal_jacobian', String, queue_size=10)

        ## Subscriber
        rospy.Subscriber('/robotis/present_joint_states', JointState, self.present_joint_states_callback)
        rospy.Subscriber('/thormang3/LinearPlanner', JointPosition, self.linearplanner_callback)
        rospy.Subscriber('/thormang3/left_arm_position', JointPosition, self.left_arm_position_callback)
        rospy.Subscriber('/thormang3/right_arm_position', JointPosition, self.right_arm_position_callback)
        
        ## set Subscriber for IK_Jacobian
        rospy.Subscriber('/thormang3/cal_jacobian_finish', JacobianMatrix, self.IK_J.cal_jacobian_finish_callback)
        
        while not rospy.is_shutdown():
            self.rate.sleep()
        

if __name__ == '__main__':
    l = LinearPlanner()
    l.run()
