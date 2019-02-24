IK_THORMANG3

put ik_thormang file in src

### ik thormang3 is depend on
cd ~/catkin_ws/src
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Math.git
git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Framework.git
git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Framework-msgs.git
git clone https://github.com/ROBOTIS-GIT/ROBOTIS-THORMANG-MPC.git
git clone https://github.com/ROBOTIS-GIT/ROBOTIS-THORMANG-Common.git
git clone https://github.com/ROBOTIS-GIT/ROBOTIS-THORMANG-msgs.git
git clone https://github.com/ROBOTIS-GIT/ROBOTIS-THORMANG-Tools.git
git clone https://github.com/ROBOTIS-GIT/ROBOTIS-THORMANG-PPC.git
git clone https://github.com/ROBOTIS-GIT/ROBOTIS-THORMANG-OPC.git

### install humanoid_navigation
sudo apt install ros-kinetic-map-server
sudo apt install ros-kinetic-humanoid-nav-msgs
sudo apt install ros-kinetic-octomap ros-kinetic-octomap-msgs ros-kinetic-octomap-ros ros-kinetic-octomap-server
sudo apt install ros-kinetic-qt-ros
sudo apt-get install ros-kinetic-ros-control
sudo apt-get install ros-kinetic-ros-controllers
sudo apt-get install ros-kinetic-gazebo-ros-control

cd ~/catkin_ws/src
git clone https://github.com/AravindaDP/humanoid_navigation.git


### How to execute
1. roslaunch thormang3_gazebo robotis_world.launch
2. roslaunch thormang3_manager thormang3_manager_gazebo.launch 
(same time robot will falling down, set the gravity to 0)
(press gazebo run button first otherwise rosnode will not show any thing)
3. roslaunch ik_thormang3 ik_thormang3_demo.launch
4. Publishes a initial position. Example: rostopic pub -1 /thormang3/LinearPlanner ik_thormang3/IK "{px: 0.43, py: 0.462, pz: 1.021, ow: 1.0, ox: 0.0, oy: 0.0, oz: 0.0, num: 100}" 
5. Go to terminal 3 and press to start and to do the tests


### pub ros node
rostopic pub -1 /thormang3/LinearPlanner ik_thormang3/IK "{px: 0.0, py: 0.0, pz: 0.0, ow: 0.0, ox: 0.0, oy: 0.0, oz: 0.0, num: 0.0}"

### px py pz         is xyz space for arm end effect
### ow ox oy oz      is orientation quaternion for arm end effect
### num              is the step number for jacobain

### more detail

i build six node in ik_thormang package 

1. /thormang3/right_arm_position
show the right arm end effect position.

2. /thormang3/left_arm_position
show the left arm end effect position.

3. /thormang3/cal_jacobian
calculate the jacobain matrix from c++ code, you can send the string left_arm and right_arm.

4. /thormang3/cal_jacobian_finish
calculate the jacobain matrix finish, and send the msg(jacobain matrix) to python code.

5. /thormang3/fk_set_joint_states
read the real robot joint position. send the value to "thormang3::KinematicsDynamics" which can calculate
the forward kinematics

6. /thormang3/LinearPlanner
this node is high level planner you can implement in python code










