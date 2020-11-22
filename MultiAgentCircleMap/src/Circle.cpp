//
// Created by alg on 16/11/20.
//

#include <MultiAgentCircleMap/Circle.h>

void Circle::setIndex(int index) {
    index_ = index;

}

void Circle::setBoolHasIndex(bool val) {
    hasIndex_ = val;

}

void Circle::pushPixelVals2stack(Eigen::Vector2d pixels) {
    pixel_coords_.push(pixels);


}

void Circle::setGlobalPosition(Eigen::Vector3d global_position) {
    global_position_ = global_position;

}

void Circle::setRadius(double radius) {
    radius_ = radius;

}
