/**
// Created by alg on 21/11/20.
Objects and Functions related to Vector of circles
 */

#ifndef MULTIAGENTCIRCLEMAP_CIRCLEVEC_H
#define MULTIAGENTCIRCLEMAP_CIRCLEVEC_H


#include <vector>
#include <opencv-3.3.1-dev/opencv2/core/matx.hpp>
#include "Circle.h"
#include "../../../../devel/include/MultiAgentCircleMap/CircleMsg.h"
#include "../../../../devel/include/MultiAgentCircleMap/CircleArray.h"


class CircleVec {
public:
    std::vector<Circle> circle_vec_;

    //default constructor
    CircleVec(){}

    //Constructor for constructing using Vec3f
    CircleVec(std::vector<cv::Vec3f> circles);

    //Constructor for constructing CircleVec using CircleArray
    CircleVec(MultiAgentCircleMap::CircleArray circle_array);



};


#endif //MULTIAGENTCIRCLEMAP_CIRCLEVEC_H
