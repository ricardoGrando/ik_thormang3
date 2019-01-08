#!/usr/bin/env python


import rospy
from ik_thormang3.msg import JointPosition,JacobianMatrix
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import numpy as np
from utils import *
import time


class IK_Jacobian:

    def __init__(self):
        self.cal_jacobian_pub = rospy.Publisher('/thormang3/cal_jacobian', String, queue_size=10)        
        self.now_cal = False
        
    def cal_jacobian_finish_callback(self,msg): 
        
        delta = self.delta
        jacobian = np.zeros(shape=(6,7))
        for i in range(6):
            for j in range(7):
                 jacobian[i][j] = msg.matrix[j+7*i]
                   
        jacobianTrans = np.dot(jacobian, np.matrix.transpose(jacobian))
        inverseJacobian = np.dot(np.matrix.transpose(jacobian), np.linalg.inv(jacobianTrans))
              
        deltaAngles = np.dot(inverseJacobian, delta) 
        
        
        
        if deltaAngles.max() < 0.1 and deltaAngles.min() > -0.1:
        
            endpose = self.curr_pose
            if self.armType == "left_arm":
                self.result = addLeftArmValue2WholeBody(deltaAngles,endpose)
            elif self.armType == "right_arm":
                self.result = addRightArmValue2WholeBody(deltaAngles,endpose)
            else:
                print("wrong arm type")
            self.now_cal = False
            
        else:
            print("error deltaAngles",deltaAngles)
            self.result = self.curr_pose
            self.now_cal = False
        
        
    def cal(self,curr_pose,delta,armType):
        self.armType = armType
        self.curr_pose = curr_pose
        self.delta  = delta
    	self.cal_jacobian_pub.publish(armType)
    	self.now_cal = True
    	
    	### wait for cal jacobain finish
        while self.now_cal:
            pass
            
            
        return self.result
    
   

