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
    //MultiAgentCircleMap::RosHandle ros_handle_;
    Graph graph_; //global graph
    CircleVec global_circles_vec_;  //global main CircleVec

    bool first_image_;

    //these are included here because of their importance for tracking. Everything else is just global here
    Image prev_image_;
    Image current_image_;

public:
    //Default constructor
    SingleRobotMapper(): first_image_(true){};

    //SingleRobotMapper(MultiAgentCircleMap::RosHandle& ros_handle):ros_handle_(ros_handle), first_image_(true){}



    std::vector<std::vector<double> > getPixelCostMatrixHungarianAlgo(double threshold_pixel_distance);
    std::vector<int> getAssignment(std::vector<std::vector<double> >& cost_matrix);
    void assignCorrepondingPrevId(std::vector<int>& assignment, std::vector<std::vector<double> > cost_matrix);
    void assignNewID();
    void updateMap(MultiAgentCircleMap::RosHandle ros_handle);
    void publishImagewithIDs(MultiAgentCircleMap::RosHandle& ros_handle);


};


#endif //MULTIAGENTCIRCLEMAP_SINGLEROBOTMAPPER_H
