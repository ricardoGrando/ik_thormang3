#include "ros/ros.h"
#include "ik_thormang3/thormang3_Kinematics.h"
#include <stdio.h>
#include <iostream>



// read current joint angle from rosnode "/robotis/present_joint_states" 
void fk_set_joint_states_Callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    for (int i = 0; i < msg->name.size(); i++)
    {
        int id = idTable.joint_name_to_id_[msg->name[i].c_str()];
        //ROS_INFO("joint %s id %d value-> %f", msg->name[i].c_str() , id , msg->position[i] );
        robotis_->thormang3_link_data_[id]->joint_angle_ = msg->position[i];
      
    }
}


void cal_jacobian_Callback(std_msgs::String msg)
{
    
    if(msg.data == "left_arm")
    {
        ik_thormang3::JacobianMatrix JacobianResult;
        float result[42];
		int ik_id_start_ = 2;
		int ik_id_end_   = 34;
		std::vector<int> idx = robotis_->findRoute(ik_id_start_, ik_id_end_);
		Eigen::MatrixXd jacobian = robotis_->calcJacobian(idx);
		for (int i = 0; i < 6; i++)
		{
			for (int j = 0; j < 7; j++)
			{
		        JacobianResult.matrix.push_back(jacobian(i,j));
			}
		}
		pub_cal_jacobian_finish.publish(JacobianResult);
    
    }
    
    if(msg.data == "right_arm")
    {
        ik_thormang3::JacobianMatrix JacobianResult;
        float result[42];
		int ik_id_start_ = 1;
		int ik_id_end_   = 33;
		std::vector<int> idx = robotis_->findRoute(ik_id_start_, ik_id_end_);
		Eigen::MatrixXd jacobian = robotis_->calcJacobian(idx);
		for (int i = 0; i < 6; i++)
		{
			for (int j = 0; j < 7; j++)
			{
		        JacobianResult.matrix.push_back(jacobian(i,j));
			}
		}
		pub_cal_jacobian_finish.publish(JacobianResult);
    
    }
} 




int main(int argc, char **argv)
{
  ros::init(argc, argv, "fk_thormang");
  ros::NodeHandle nh;
  
  ros::Rate loop_rate(60);
  
  ROS_INFO("run cal_fk_thormang node");
  
  // initize robotis_ from ~/ROBOTIS-THORMANG-MPC/thormang3_kinematics_dynamics/src/kinematics_dynamics.cpp
  // this file build the thormang3 kinematics object
  robotis_ = new thormang3::KinematicsDynamics(thormang3::WholeBody);
   
  // SUBSCRIBER
  sub_curr_joint_angle = nh.subscribe("/thormang3/fk_set_joint_states", 1000, fk_set_joint_states_Callback);
  cal_jacobian = nh.subscribe("/thormang3/cal_jacobian", 1000, cal_jacobian_Callback);
  
  // PUBLISHER
  pub_left_arm_position = nh.advertise<ik_thormang3::JointPosition>("/thormang3/left_arm_position", 1000);
  pub_right_arm_position = nh.advertise<ik_thormang3::JointPosition>("/thormang3/right_arm_position", 1000);
  pub_cal_jacobian_finish = nh.advertise<ik_thormang3::JacobianMatrix>("/thormang3/cal_jacobian_finish", 1000);
    

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
     curr_position(0) = curr_position(0);  
     curr_position(1) = curr_position(1);   
     curr_position(2) = curr_position(2);  
     
     // left arm orientation_
     curr_rotation = robotis_->thormang3_link_data_[34]->orientation_;
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
     curr_position(0) = curr_position(0);
     curr_position(1) = curr_position(1);
     curr_position(2) = curr_position(2);
     
     // left arm orientation_
     curr_rotation = robotis_->thormang3_link_data_[33]->orientation_;
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

