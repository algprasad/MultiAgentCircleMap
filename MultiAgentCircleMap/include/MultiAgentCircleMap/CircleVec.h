/**
// Created by alg on 21/11/20.
Objects and Functions related to Vector of circles
 */

#ifndef MULTIAGENTCIRCLEMAP_CIRCLEVEC_H
#define MULTIAGENTCIRCLEMAP_CIRCLEVEC_H


#include <vector>
#include <opencv-3.3.1-dev/opencv2/core/matx.hpp>
#include "Circle.h"

class CircleVec {
public:
    //default constructor
    CircleVec(){}

    //Constructor for constructing using Vec3f
    CircleVec(std::vector<cv::Vec3f> circles){
        for(int i =0; i< circles.size(); i++){
            Circle temp_circle;
            temp_circle.setIndex(i+1);
            temp_circle.setBoolHasIndex(false);
            temp_circle.pushPixelVals2stack(Eigen::Vector2d(circles[i][0], circles[i][1]));
            temp_circle.setRadius(circles[i][2]);
            circle_vec_.push_back(temp_circle);

        }
    }


    std::vector<Circle> circle_vec_;
};


#endif //MULTIAGENTCIRCLEMAP_CIRCLEVEC_H
