#!/usr/bin/env python


import rospy
from ik_thormang3.msg import JointPosition,JacobianMatrix
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import numpy as np
from utils import *
import time
import getpass
from keras.models import load_model
from keras.models import Model

class IK_Jacobian:

    def __init__(self):
        self.cal_jacobian_pub = rospy.Publisher('/thormang3/cal_jacobian', String, queue_size=10)        
        self.now_cal = False

        self.model = load_model("/home/"+getpass.getuser()+"/catkin_ws/src/ik_thormang3/model__340_2048_Adam_sigmoid_150_120_90_75_65_55_7_10000000_0.013154525557590856_0.01312057321594821_0.0007109376021267358.h5")
        self.model._make_predict_function()

        # Its limits for normalization/desnormalization
        self.highestList = np.array([   0.0996578,  0.09700995, 0.09993384, 0.33062578, 0.28312842, 0.31182562,\
                                        0.27920513, 1.59999821, 1.59999794, 1.59999969, 1.29999984, 2.79999662,\
                                        1.3999988 , 1.3999999 , 0.08726649, 0.08726648, 0.0872665,  0.0872665,\
                                        0.08726646, 0.0872665,  0.08726644])

        self.lowestList = np.array([    -0.09980583, -0.09704175, -0.09984278, -0.31763874, -0.28009778, -0.31739168,\
                                        -0.28857708, -1.59999977, -1.59999941, -1.59999798, -1.29999983, -2.79999956,\
                                        -1.39999721, -1.39999988, -0.08726647, -0.08726649, -0.0872665,  -0.08726644,\
                                        -0.08726649, -0.08726646, -0.08726647])
        
    def cal_jacobian_finish_callback(self,msg):

        delta = self.delta 

        if self.net:
            predictArray = np.array([   delta[0],
                                            delta[1],
                                            delta[2],
                                            delta[3],
                                            delta[4],
                                            delta[5],
                                            delta[6],                             
                                            self.curr_pose['l_arm_sh_p1'],
                                            self.curr_pose['l_arm_sh_r'],
                                            self.curr_pose['l_arm_sh_p2'],
                                            self.curr_pose['l_arm_el_y'],
                                            self.curr_pose['l_arm_wr_r'],
                                            self.curr_pose['l_arm_wr_y'],
                                            self.curr_pose['l_arm_wr_p']                                
                                        ])
            # Normalize the predict array
            predictArray = normalizeArray(predictArray, self.highestList[:14], self.lowestList[:14])
            # The ANN output
            deltaAngles = (np.array(self.model.predict(predictArray.reshape(1,14)))[0])  
            # Desnormalize the predicted angles
            deltaAngles = desnormalizeArray(deltaAngles, self.highestList[14:], self.lowestList[14:])
        else:
        
            jacobian = np.zeros(shape=(6,7))
            for i in range(6):
                for j in range(7):
                    jacobian[i][j] = msg.matrix[j+7*i]
                    
            jacobianTrans = np.dot(jacobian, np.matrix.transpose(jacobian))
            inverseJacobian = np.dot(np.matrix.transpose(jacobian), np.linalg.inv(jacobianTrans))
                
            deltaAngles = np.dot(inverseJacobian, delta)

        max_delta = 10 
        
        if deltaAngles.max() < max_delta and deltaAngles.min() > -max_delta:
        
            endpose = self.curr_pose
            if self.armType == "left_arm":
                self.result = addLeftArmValue2WholeBody(deltaAngles,endpose)
            elif self.armType == "right_arm":
                self.result = addRightArmValue2WholeBody(deltaAngles,endpose)
            else:
                print("wrong arm type")
            self.now_cal = False
            
        else:
            #print("error deltaAngles",deltaAngles)
            self.result = self.curr_pose
            self.now_cal = False
        
        
    def cal(self,curr_pose,delta,armType,net):
        self.armType = armType
        self.curr_pose = curr_pose
        self.delta  = delta
    	self.cal_jacobian_pub.publish(armType)
    	self.now_cal = True
        self.net = net
    	
    	### wait for cal jacobain finish
        while self.now_cal:
            pass
            
            
        return self.result
    
   

