#!/usr/bin/env python

import math
import numpy as np

def normalizeArray(list, highestList, lowestList):
    newList = np.zeros(shape=(14,1))
    for i in range(0, list.shape[0]):
        #print (list[i])
        #print (highestList[i])
        newList[i] = ((list[i] - lowestList[i])/(highestList[i] - lowestList[i]))*(0.9-(0.1)) + 0.1 
                    
    return newList
    
def desnormalizeArray(list, highestList, lowestList):
    newList = np.zeros(shape=(7,1))
    for i in range(0, list.shape[0]):
        newList[i] = ((list[i]-0.1)*(highestList[i] - lowestList[i])/((0.9-(0.1))) + lowestList[i])
                
    return newList

def get_left_arm_pose(joint_pose):
    left_arm_name = ['l_arm_sh_p1','l_arm_sh_r','l_arm_sh_p2','l_arm_el_y','l_arm_wr_r','l_arm_wr_y','l_arm_wr_p']
    left_arm_pose = []
    for name in left_arm_name:
        left_arm_pose.append((name,joint_pose[name]))
        
    return left_arm_pose
    
def addLeftArmValue2WholeBody(l_arm,whole):

    left_arm_name = ['l_arm_sh_p1','l_arm_sh_r','l_arm_sh_p2','l_arm_el_y','l_arm_wr_r','l_arm_wr_y','l_arm_wr_p']
    left_arm_dict = {'l_arm_sh_p1':0,'l_arm_sh_r':1,'l_arm_sh_p2':2,'l_arm_el_y':3,'l_arm_wr_r':4,'l_arm_wr_y':5,'l_arm_wr_p':6}
    for name in left_arm_name:
        whole[name] += l_arm[left_arm_dict[name]]
    return whole
    
    
def addRightArmValue2WholeBody(r_arm,whole):

    right_arm_name = ['r_arm_sh_p1','r_arm_sh_r','r_arm_sh_p2','r_arm_el_y','r_arm_wr_r','r_arm_wr_y','r_arm_wr_p']
    right_arm_dict = {'r_arm_sh_p1':0,'r_arm_sh_r':1,'r_arm_sh_p2':2,'r_arm_el_y':3,'r_arm_wr_r':4,'r_arm_wr_y':5,'r_arm_wr_p':6}
    for name in right_arm_name:
        whole[name] += r_arm[right_arm_dict[name]]
    return whole
    
def angle_limit(a):
    return math.atan2(math.sin(a),math.cos(a))
    
def cal_rpy_delta(end,curr):
    x = end[0] - curr[0]
    y = end[1] - curr[1]
    z = end[2] - curr[2]
    
    c_r,c_p,c_y = quaternion_to_euler_angle(curr[4], curr[5], curr[6], curr[3])
    e_r,e_p,e_y = quaternion_to_euler_angle(end[4], end[5], end[6], end[3])
    
    ar = angle_limit(e_r - c_r)
    ap = angle_limit(e_p - c_p)
    ay = angle_limit(e_y - c_y)
    
    
    return np.array([x,y,z,ar,ap,ay])
    
    
    
def quaternion_to_euler_angle(x, y, z, w):
	ysqr = y * y
	
	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + ysqr)
	X = math.atan2(t0, t1)
	
	t2 = +2.0 * (w * y - z * x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	Y = math.asin(t2)
	
	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (ysqr + z * z)
	Z = math.atan2(t3, t4)
	
	return angle_limit(X), angle_limit(Y), angle_limit(Z)
