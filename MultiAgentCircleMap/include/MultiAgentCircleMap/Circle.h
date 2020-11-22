//
// Created by alg on 21/11/20.
//

#ifndef MULTIAGENTCIRCLEMAP_CIRCLE_H
#define MULTIAGENTCIRCLEMAP_CIRCLE_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <stack>

class Circle {
public:
    int index_;
    double radius_;
    bool hasIndex_;
    std::stack<Eigen::Vector2d> pixel_coords_;
    Eigen::Vector3d global_position_;

public:

    //default constructor
    Circle(){
        index_ = 0;
        hasIndex_ = false;
        pixel_coords_.push(Eigen::Vector2d(0, 0));
        global_position_ << 0, 0, 0;
    }


    void setRadius(double radius);
    void setIndex(int index);
    void setBoolHasIndex(bool val);
    void pushPixelVals2stack(Eigen::Vector2d pixels);
    void setGlobalPosition(Eigen::Vector3d global_position);


};


#endif //MULTIAGENTCIRCLEMAP_CIRCLE_H
