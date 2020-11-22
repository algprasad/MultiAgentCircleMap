/**
// Created by alg on 21/11/20.
Main class for the algorithm for mapping with single robot (Local Map)
 */

#ifndef MULTIAGENTCIRCLEMAP_SINGLEROBOTMAPPER_H
#define MULTIAGENTCIRCLEMAP_SINGLEROBOTMAPPER_H


#include "RosHandle.hpp"
#include "Graph.h"
#include "CircleVec.h"

class SingleRobotMapper {
    MultiAgentCircleMap::RosHandle ros_handle_;
    Graph graph_; //global graph
    CircleVec circle_vec_;  //global main CircleVec

    //these are included here because of their importance for tracking. Everything else is just global here
    Image prev_image_;
    Image current_image_;

public:
    SingleRobotMapper(MultiAgentCircleMap::RosHandle ros_handle):ros_handle_(ros_handle){}



};


#endif //MULTIAGENTCIRCLEMAP_SINGLEROBOTMAPPER_H
