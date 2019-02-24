#!/usr/bin/env python


import rospy
from ik_thormang3.msg import JointPosition,JacobianMatrix,IK
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import numpy as np
from utils import *
from ik_jacobian import *
from std_msgs.msg import Float64
from gazebo_msgs.srv import SpawnModel
import geometry_msgs.msg
from random import randint
import getpass
from keras.models import load_model
from keras.models import Model

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

    def spiral(self, stepSize, radius, endEffectorPosition, numberSpirals):
        endEffector = np.zeros(shape=(int(2*math.pi*radius/stepSize)*numberSpirals,7))
        endEffector[:] = endEffectorPosition
            
        angle = math.asin(stepSize/(radius))
        
        for i in range(0, endEffector.shape[0]):
            endEffector[i][2] = endEffector[i][2] - radius*stepSize*(i+1)       
            endEffector[i][0] = endEffector[i][0] + radius*math.cos(angle*(i+1)) - radius
            endEffector[i][1] = endEffector[i][1] + radius*math.sin(angle*(i+1))
            
            if i > 0:
                dx = (endEffector[i][0] - endEffector[i-1][0])
                dy = (endEffector[i][1] - endEffector[i-1][1])
                dz = (endEffector[i][2] - endEffector[i-1][2])
                d =  ( dx**2 + dy**2 + dz**2 )**(0.5)                
        
        return endEffector, d

    def writeToFile(self, name, calculated, desired, angles):
        file = open(name+".txt","a")
        string = str(calculated[0])+','+str(calculated[1])+','+str(calculated[2])+','\
                +str(calculated[3])+','+str(calculated[4])+','+str(calculated[5])+','+str(calculated[6])+','\
                +str(desired[0])+','+str(desired[1])+','+str(desired[2])+','+str(desired[3])+','+str(desired[4])+','\
                +str(desired[5])+','+str(desired[6])+','\
                +str(angles[0])+','+str(angles[1])+','+str(angles[2])+','+str(angles[3])+','\
                +str(angles[4])+','+str(angles[5])+','+str(angles[6])

        file.write(str(string+'\n'))
        file.close()

    def writeDistanceToFile(self, name, d):
        file = open(name+".txt","w")    
        file.write(str(d)+'\n')
        file.close()

    def publishOnGazeboTopics(self, result, curr, trajectoryFlag):
        #for i in range(0,10):
        self.pub = rospy.Publisher(pubList[0], Float64, queue_size=100)
        self.pub.publish(Float64(result['l_arm_sh_p1']))
        self.pub = rospy.Publisher(pubList[1], Float64, queue_size=100)
        self.pub.publish(Float64(result['l_arm_sh_r']))
        self.pub = rospy.Publisher(pubList[2], Float64, queue_size=100)
        self.pub.publish(Float64(result['l_arm_sh_p2']))
        self.pub = rospy.Publisher(pubList[3], Float64, queue_size=100)
        self.pub.publish(Float64(result['l_arm_el_y']))
        self.pub = rospy.Publisher(pubList[4], Float64, queue_size=100)
        self.pub.publish(Float64(result['l_arm_wr_r']))
        self.pub = rospy.Publisher(pubList[5], Float64, queue_size=100)
        self.pub.publish(Float64(result['l_arm_wr_y']))
        self.pub = rospy.Publisher(pubList[6], Float64, queue_size=100)
        self.pub.publish(Float64(result['l_arm_wr_p']))

        # if trajectoryFlag:
        #     self.spawnObject(curr[0], curr[1], curr[2])
        # else:
        #     pass

    def controlRobot(self, result):
        ### control the robot
        joint = JointState()
        joint.name = result.keys()
        joint.position = result.values()
        self.set_joint_pub.publish(joint)

    def getArmEndEffectorCartesianPosition(self, arm_type):
         ### get current end position
        if arm_type == "left_arm":
            curr = self.left_arm_position.copy()
        elif arm_type == "right_arm":
            curr = self.right_arm_position.copy()

        return curr

    def setInitialPosition(self, arm_type, num):

        arr = [] 

        ### get each step value for linearplanner 
    	if arm_type == "left_arm":
    	    for i in range(7):
    	        arr.append(np.linspace(self.left_arm_position[i],self.target_pose[i],num))
    	elif arm_type == "right_arm":
    	    for i in range(7):
    	        arr.append(np.linspace(self.right_arm_position[i],self.target_pose[i],num))
     	
        arr = np.array(arr).T
        
    	for j in range(1,num):
    	    
    	    curr = self.getArmEndEffectorCartesianPosition(arm_type)
    	        
    	    ### calulate the delta in rpy space
    	    delta = cal_rpy_delta(arr[j] , curr)
            
    	    result = self.IK_J.cal(self.joint_pose, delta, arm_type, False)            
            
            self.publishOnGazeboTopics(result, curr, False)

            self.controlRobot(result)

    def spawnObject(self, posx, posy, posz):
        e = 0
        try:
            spawn = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

            rospy.wait_for_service("gazebo/spawn_sdf_model")

            initial_pose = geometry_msgs.msg.Pose()
            initial_pose.position.x = posx
            initial_pose.position.y = posy
            initial_pose.position.z = posz+0.18

            f = open('/home/ricardo/.gazebo/models/cube_spiral/model.sdf','r')
            sdff = f.read()

            print spawn("cube"+str(randint(0, 100000)), sdff, "default", initial_pose, "world")

        except (rospy.ServiceException, e):
            print ("Deu ruim: %s"%e)

    def linearplanner_callback(self,msg):
        self.target_pose = np.array([msg.px, msg.py, msg.pz, msg.ow, msg.ox, msg.oy, msg.oz])
    	        
        arm_type = "left_arm"

        print("Press to start")
        raw_input()

        ##############################################################
        # Step size to create the trajectory(Desired distance between each point)
        step = 0.03
        # Size to increment after the test with the 'step'
        step_size = 0.001
        # Radius on the spiral
        spiral_radius = 0.1
        # Number of spirals
        number_spirals = 2

        while(step < spiral_radius):

            print("Going to initial position to start the test with jacobian....")
            self.setInitialPosition(arm_type, int(msg.num))
            print("Initial position reached")

            ### get current end position
            if arm_type == "left_arm":
                curr = self.left_arm_position.copy()
            elif arm_type == "right_arm":
                curr = self.right_arm_position.copy()
                
            trajectory, d = self.spiral(step, spiral_radius, curr, number_spirals)
            
            i = 0

            # Saves the real distance of eacj point in the trajectory
            self.writeDistanceToFile("/home/"+getpass.getuser()+"/catkin_ws/src/ik_thormang3/outputs/jac/jac_"+str(step), d)

            # Goes through each point of the trajectory
            while(i < trajectory.shape[0]-1):

                print("Press to go to the next point in the trajectory")
                raw_input()

                if arm_type == "left_arm":
                    curr = self.left_arm_position.copy()
                elif arm_type == "right_arm":
                    curr = self.right_arm_position.copy()
                
                angles = np.array([self.joint_pose['l_arm_sh_p1'], self.joint_pose['l_arm_sh_r'], self.joint_pose['l_arm_sh_p2'], self.joint_pose['l_arm_el_y'], self.joint_pose['l_arm_wr_r'], self.joint_pose['l_arm_wr_y'], self.joint_pose['l_arm_wr_p']])
                
                # Saves the error based of where it goes(curr) and where it should have gone(trajectory[i-1])
                if i > 0:
                    # print("Curr position: "+str(curr))
                    # print("Desired position: "+str(trajectory[i-1]))
                    
                    self.writeToFile("/home/"+getpass.getuser()+"/catkin_ws/src/ik_thormang3/outputs/jac/jac_"+str(step), curr, trajectory[i-1], angles)

                # Calculates the delta
                delta = cal_rpy_delta(trajectory[i], curr)

                # Calls the ik with the jacobian(False)
                result = self.IK_J.cal(self.joint_pose, delta, arm_type, False)
                
                # print("###########################################")
                # print("Delta End Effector"+str(trajectory[i+1]-self.left_arm_position))
                # print("Delta Angles: "+str(delta))
                # print("Angles: ")
                # print(str(angles))
                # print("EndEffector: "+str(self.left_arm_position))
                # print("###########################################")

                # Publishes on the gazebo topics
                self.publishOnGazeboTopics(result, curr, True)

                # Control the robot
                self.controlRobot(result)

                i += 1

            # print("###########################################")
            print("Finished trajectory with jacobian with step "+str(d))

            print("Going to initial position to start the test with net....")
            self.setInitialPosition(arm_type, int(msg.num))
            print("Initial position reached")

            ### get current end position
            if arm_type == "left_arm":
                curr = self.left_arm_position.copy()
            elif arm_type == "right_arm":
                curr = self.right_arm_position.copy()                
            
            i = 0

            # Saves the distance between each point of the trajectory in the file
            self.writeDistanceToFile("/home/"+getpass.getuser()+"/catkin_ws/src/ik_thormang3/outputs/net/net_"+str(step), d)

            while(i < trajectory.shape[0]-1):

                print("Press to go to the next point in the trajectory")
                raw_input()

                if arm_type == "left_arm":
                    curr = self.left_arm_position.copy()
                elif arm_type == "right_arm":
                    curr = self.right_arm_position.copy()
                
                angles = np.array([self.joint_pose['l_arm_sh_p1'], self.joint_pose['l_arm_sh_r'], self.joint_pose['l_arm_sh_p2'], self.joint_pose['l_arm_el_y'], self.joint_pose['l_arm_wr_r'], self.joint_pose['l_arm_wr_y'], self.joint_pose['l_arm_wr_p']])

                if i > 0:
                    # print("Curr position: "+str(curr))
                    # print("Desired position: "+str(trajectory[i-1]))
                    
                    self.writeToFile("/home/"+getpass.getuser()+"/catkin_ws/src/ik_thormang3/outputs/net/net_"+str(step), curr, trajectory[i-1], angles)

                # Calculates the delta
                delta = trajectory[i] - curr              

                # Makes the ik using the net(True)
                result = self.IK_J.cal(self.joint_pose, delta, arm_type, True)
                
                # print("###########################################")
                # print("Delta End Effector"+str(trajectory[i+1]-self.left_arm_position))
                # print("Delta Angles: "+str(delta))
                # print("Angles: ")
                # print(str(angles))
                # print("EndEffector: "+str(self.left_arm_position))
                # print("###########################################")

                # Publishes on gazebo topics
                self.publishOnGazeboTopics(result, curr, True)

                # Control the robot
                self.controlRobot(result)

                i += 1

            # print("###########################################")
            print("Finished trajectory with net with step "+str(d))           

            print("Press to increase the step size")            
            raw_input()

            step += step_size

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
