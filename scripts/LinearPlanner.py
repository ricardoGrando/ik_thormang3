#!/usr/bin/env python


import rospy
from ik_thormang3.msg import JointPosition,JacobianMatrix,IK
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import numpy as np
from utils import *
from ik_jacobian import *
from std_msgs.msg import Float64

pubList =  [    '/thormang3/l_arm_sh_p1_position/command', # -1.6 to 1.6
                '/thormang3/l_arm_sh_r_position/command', # -1.6 to 1.6
                '/thormang3/l_arm_sh_p2_position/command', # -1.6 to 1.6
                '/thormang3/l_arm_el_y_position/command',  # -1.3 to 1.3
                '/thormang3/l_arm_wr_r_position/command', # -2.8 to 2.8
                '/thormang3/l_arm_wr_y_position/command', # 1.4 to 1.4
                '/thormang3/l_arm_wr_p_position/command', # -1.4 to 1.4                           
            ]

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
    	
    	arm_type = "left_arm"

        arr = []
    	
    	### step num
    	num = int(msg.num)
        #print(num)
    	
    	### get each step value for linearplanner 
    	if arm_type == "left_arm":
    	    for i in range(7):
    	        arr.append(np.linspace(self.left_arm_position[i],self.target_pose[i],num))
    	elif arm_type == "right_arm":
    	    for i in range(7):
    	        arr.append(np.linspace(self.right_arm_position[i],self.target_pose[i],num))
     	
        #print(arr)
    		
    	arr = np.array(arr).T
        print(arr)
    	
    	for j in range(1,num):
    	    
    	    ### get current end position
    	    if arm_type == "left_arm":
    	        curr = self.left_arm_position.copy()
    	    elif arm_type == "right_arm":
    	        curr = self.right_arm_position.copy()
    	        
    	    ### calulate the delta in rpy space
    	    delta = cal_rpy_delta(arr[j] , curr)
            print("###########################################")
            print("Delta: "+str(delta))
    	    
    	    ### calculate ik jacobain or nn jacobain
    	    # TODO 
            #print(self.joint_pose)
    	    result = self.IK_J.cal(self.joint_pose, delta, arm_type)
            print("###########################################")
            print("Delta Angles: ")
            print(self.joint_pose['l_arm_sh_p1'], self.joint_pose['l_arm_sh_r'], self.joint_pose['l_arm_sh_p2'], self.joint_pose['l_arm_el_y'], self.joint_pose['l_arm_wr_r'], self.joint_pose['l_arm_wr_y'], self.joint_pose['l_arm_wr_p'])
    	    print("###########################################")
            print("EndEffector: "+str(self.left_arm_position))
            print("###########################################")
            print(result['l_arm_sh_p1'])
            
            self.pub = rospy.Publisher(pubList[0], Float64, queue_size=10)
            self.pub.publish(Float64(result['l_arm_sh_p1']))
            self.pub = rospy.Publisher(pubList[1], Float64, queue_size=10)
            self.pub.publish(Float64(result['l_arm_sh_r']))
            self.pub = rospy.Publisher(pubList[2], Float64, queue_size=10)
            self.pub.publish(Float64(result['l_arm_sh_p2']))
            self.pub = rospy.Publisher(pubList[3], Float64, queue_size=10)
            self.pub.publish(Float64(result['l_arm_el_y']))
            self.pub = rospy.Publisher(pubList[4], Float64, queue_size=10)
            self.pub.publish(Float64(result['l_arm_wr_r']))
            self.pub = rospy.Publisher(pubList[5], Float64, queue_size=10)
            self.pub.publish(Float64(result['l_arm_wr_y']))
            self.pub = rospy.Publisher(pubList[6], Float64, queue_size=10)
            self.pub.publish(Float64(result['l_arm_wr_p']))

            #print(result)
    	    ### control the robot
    	    joint = JointState()
            joint.name = result.keys()
            joint.position = result.values()
            self.set_joint_pub.publish(joint)

            #raw_input()
      
    def present_joint_states_callback(self,msg):
        self.joint_pose = dict(zip(msg.name, msg.position))
        self.fk_pub.publish(msg)
        #pass
        #print(self.joint_pose)
        
    def left_arm_position_callback(self,msg):

        self.left_arm_position = np.array([msg.px, msg.py, msg.pz, msg.ow, msg.ox, msg.oy, msg.oz])
    	
    def right_arm_position_callback(self,msg):
        self.right_arm_position = np.array([msg.px, msg.py, msg.pz, msg.ow, msg.ox, msg.oy, msg.oz])
        
    def run(self):
        print("run LinearPlanner")
        rospy.init_node('linearplanner', anonymous=True)
        self.rate = rospy.Rate(60)
        
        ## Publisher
        self.set_joint_pub = rospy.Publisher('/robotis/set_joint_states', JointState, queue_size=10)
        self.fk_pub = rospy.Publisher('/thormang3/fk_set_joint_states', JointState, queue_size=10)
      
        ## Subscriber
        rospy.Subscriber('/robotis/present_joint_states', JointState, self.present_joint_states_callback)
        rospy.Subscriber('/thormang3/LinearPlanner', IK, self.linearplanner_callback)
        rospy.Subscriber('/thormang3/left_arm_position', JointPosition, self.left_arm_position_callback)
        rospy.Subscriber('/thormang3/right_arm_position', JointPosition, self.right_arm_position_callback)
        
        ## set Subscriber for IK_Jacobian
        rospy.Subscriber('/thormang3/cal_jacobian_finish', JacobianMatrix, self.IK_J.cal_jacobian_finish_callback)
        
        while not rospy.is_shutdown():
            self.rate.sleep()
        

if __name__ == '__main__':
    np.set_printoptions(suppress=True) 
    l = LinearPlanner()
    l.run()
