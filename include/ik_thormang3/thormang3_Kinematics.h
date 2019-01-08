#ifndef THORMANG3_KINEMATICS_H
#define THORMANG3_KINEMATICS_H

#include <vector>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "thormang3_manipulation_module_msgs/JointPose.h"
#include "thormang3_manipulation_module_msgs/KinematicsPose.h"
#include "thormang3_kinematics_dynamics/kinematics_dynamics.h"
#include "ik_thormang3/IDTable.h"
#include "ik_thormang3/JointPosition.h"
#include "ik_thormang3/JacobianMatrix.h"
thormang3::KinematicsDynamics *robotis_;
IDTable idTable;

static ros::Subscriber sub_curr_joint_angle;
static ros::Subscriber cal_jacobian;

static ros::Publisher pub_left_arm_position;
static ros::Publisher pub_right_arm_position;
static ros::Publisher pub_cal_jacobian_finish;

#endif

