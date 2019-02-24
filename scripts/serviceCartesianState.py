import sys
import rospy
import roslib
from std_msgs.msg import String
import threading
from gazebo_msgs.srv import GetLinkState 
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import SetLinkState 
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import LinkState # For getting information about link states
import time
from std_msgs.msg import Float64
from gazebo_msgs.srv import SpawnModel
import geometry_msgs.msg

"""
    Description:    Gets position e orientation of a specified link 
"""

class serviceCartesianState(object):
    def __init__ (self, getService, linkOrModel, world, mode):
        self.getService = getService
        self.linkOrModel = linkOrModel   
        self.world = world
        self.mode = mode
        
        self.pos_x = 0
        self.pos_y = 0
        self.pos_z = 0
        self.ori_x = 0
        self.ori_y = 0
        self.ori_z = 0

        self.pos_x_last = 0
        self.pos_y_last = 0
        self.pos_z_last = 0
        self.ori_x_last = 0
        self.ori_y_last = 0
        self.ori_z_last = 0

    def __treatLinkState__(self, linkState):
        if self.mode == "model":
            offset = 4 
            offset1 = 0
        else:
            offset = 0 
            offset1 = 1      

        a = linkState.split('\n')
        
        self.pos_x_last = self.pos_x
        self.pos_y_last = self.pos_y
        self.pos_z_last = self.pos_z
        self.ori_x_last = self.ori_x
        self.ori_y_last = self.ori_y
        self.ori_z_last = self.ori_z

        self.pos_x = float(a[4+offset][7+offset1:])
        self.pos_y = float(a[5+offset][7+offset1:])
        self.pos_z = float(a[6+offset][7+offset1:])

        self.ori_x = float(a[8+offset][7+offset1:])
        self.ori_y = float(a[9+offset][7+offset1:])
        self.ori_z = float(a[10+offset][7+offset1:])
                
        print (self.linkOrModel+": "+str(self.pos_x)+" "+str(self.pos_y)+" "+str(self.pos_z)+" "+ \
                                    str(self.ori_x)+" "+str(self.ori_y)+" "+str(self.ori_z))
                
    def getState(self):        
        try:
            if self.mode == "model":
                info_prox = rospy.ServiceProxy(self.getService, GetModelState)
            else:
                info_prox = rospy.ServiceProxy(self.getService, GetLinkState)
            rospy.wait_for_service(self.getService)
            
            self.__treatLinkState__(str(info_prox( self.linkOrModel , self.world )))
                
        except (rospy.ServiceException, e):
            print ("Deu ruim: %s"%e)

    def spawnObject(self, count, posx, posy, posz):
        e = 0
        try:
            spawn = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

            rospy.wait_for_service("gazebo/spawn_sdf_model")

            initial_pose = geometry_msgs.msg.Pose()
            initial_pose.position.x = posx
            initial_pose.position.y = posy
            initial_pose.position.z = posz

            f = open('/home/ricardo/.gazebo/models/cube_spiral/model.sdf','r')
            sdff = f.read()

            print spawn("cube"+str(count), sdff, "default", initial_pose, "world")

        except (rospy.ServiceException, e):
            print ("Deu ruim: %s"%e)
    
    

            