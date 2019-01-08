#ifndef FOREARD_KINEMATICS_H
#define FOREARD_KINEMATICS_H

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

thormang3::KinematicsDynamics *robotis_;
IDTable idTable;

#endif

