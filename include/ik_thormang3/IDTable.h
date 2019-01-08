#ifndef JOINT_NAME_TO_ID_H
#define JOINT_NAME_TO_ID_H

#include <stdio.h>
#include <iostream>
#include <map>


class IDTable{

public:
  IDTable();
  
  std::map<std::string, int> joint_name_to_id_;
  
};
#endif

