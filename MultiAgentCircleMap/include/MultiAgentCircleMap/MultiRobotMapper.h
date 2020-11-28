//
// Created by alg on 21/11/20.
//

#ifndef MULTIAGENTCIRCLEMAP_MULTIROBOTMAPPER_H
#define MULTIAGENTCIRCLEMAP_MULTIROBOTMAPPER_H

#include "RosHandle.hpp"
#include "SingleRobotMapper.h"

class MultiRobotMapper {
    int robot_index_;


public:
    //default constructor
    MultiRobotMapper(int robot_index):robot_index_(robot_index){};

    void updateMap(MultiAgentCircleMap::RosHandle ros_handle, SingleRobotMapper& single_robot_mapper);

};


#endif //MULTIAGENTCIRCLEMAP_MULTIROBOTMAPPER_H
