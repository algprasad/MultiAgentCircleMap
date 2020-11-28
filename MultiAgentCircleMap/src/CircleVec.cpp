//
// Created by alg on 21/11/20.
//

#include "MultiAgentCircleMap/CircleVec.h"

CircleVec::CircleVec(MultiAgentCircleMap::CircleArray circle_array) {
    for(int i =0; i< circle_array.circle_vec.size(); i++){
        Circle temp_circle;
        temp_circle.radius_ = circle_array.circle_vec[i].radius;

        temp_circle.global_position_ = Eigen::Vector3d(circle_array.circle_vec[i].centre_x,
                                                       circle_array.circle_vec[i].centre_y,
                                                       circle_array.circle_vec[i].centre_z);
        temp_circle.setBoolHasIndex(false);
        this->circle_vec_.push_back(temp_circle);
    }


}

CircleVec::CircleVec(std::vector<cv::Vec3f> circles) {
    for(int i =0; i< circles.size(); i++){
        Circle temp_circle;
        temp_circle.setIndex(i+1);
        temp_circle.setBoolHasIndex(false);
        temp_circle.pushPixelVals2stack(Eigen::Vector2d(circles[i][0], circles[i][1]));
        temp_circle.setRadius(circles[i][2]);
        circle_vec_.push_back(temp_circle);

    }

}
