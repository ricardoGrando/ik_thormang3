#include "ros/ros.h"
#include "ik_thormang3/forward_kinematics.h"
#include <stdio.h>
#include <iostream>



// read current joint angle from rosnode "/robotis/present_joint_states" 
void curr_joint_angle_Callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    for (int i = 0; i < msg->name.size(); i++)
    {
        int id = idTable.joint_name_to_id_[msg->name[i].c_str()];
        //ROS_INFO("joint %s id %d value-> %f", msg->name[i].c_str() , id , msg->position[i] );
        robotis_->thormang3_link_data_[id]->joint_angle_ = msg->position[i];
      
    }
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "fk_thormang");
  ros::NodeHandle nh;
  
  ros::Rate loop_rate(10);
  
  ROS_INFO("run fk_thormang node");
  
  // initize robotis_ from ~/ROBOTIS-THORMANG-MPC/thormang3_kinematics_dynamics/src/kinematics_dynamics.cpp
  // this file build the thormang3 kinematics object
  robotis_ = new thormang3::KinematicsDynamics(thormang3::WholeBody);
   
  // %Tag(SUBSCRIBER)%
  ros::Subscriber sub_curr_joint_angle = nh.subscribe("/robotis/present_joint_states", 1000, curr_joint_angle_Callback);
  ros::Publisher pub_left_arm_position = nh.advertise<ik_thormang3::JointPosition>("/thormang3/left_arm_position", 1000);
  ros::Publisher pub_right_arm_position = nh.advertise<ik_thormang3::JointPosition>("/thormang3/right_arm_position", 1000);
    

  Eigen::MatrixXd curr_rotation;
  Eigen::Quaterniond q;
  ik_thormang3::JointPosition msg;
  
  
  while (ros::ok())
  {
  
     // call ForwardKinematic from thormang3_base
     robotis_->calcForwardKinematics(0);
     
     
     // left arm XYZ position
     Eigen::MatrixXd curr_position = robotis_->thormang3_link_data_[34]->position_;
     // keep the same value to thormang3_demo
     curr_position(0) = curr_position(0); //+ 0.046636;  
     curr_position(1) = curr_position(1);  //+ 0.020177;   
     curr_position(2) = curr_position(2);  //- 0.143068;  
     
     // left arm orientation_
     curr_rotation = robotis_->thormang3_link_data_[14]->orientation_;
     q = robotis_framework::convertRotationToQuaternion(curr_rotation);

     // publish left arm XYZ position and orientation 
     msg.px = curr_position(0);
     msg.py = curr_position(1);
     msg.pz = curr_position(2); 
     msg.ow = q.w();
     msg.ox = q.x();
     msg.oy = q.y();
     msg.oz = q.z();
     pub_left_arm_position.publish(msg);
    
     
     
     curr_position = robotis_->thormang3_link_data_[33]->position_;
     // keep the same value to thormang3_demo
     curr_position(0) = curr_position(0) + 0.046636;   
     curr_position(1) = curr_position(1) - 0.020177;  
     curr_position(2) = curr_position(2) - 0.143068; 
     
     // left arm orientation_
     curr_rotation = robotis_->thormang3_link_data_[13]->orientation_;
     q = robotis_framework::convertRotationToQuaternion(curr_rotation);
     
     // publish right arm XYZ position and orientation
     msg.px = curr_position(0);
     msg.py = curr_position(1);
     msg.pz = curr_position(2); 
     msg.ow = q.w();
     msg.ox = q.x();
     msg.oy = q.y();
     msg.oz = q.z();
     pub_right_arm_position.publish(msg);
       
     
     ros::spinOnce();
     loop_rate.sleep();   
  }
  
  return 0;
}

