//
// Created by alg on 27/11/20.
//

#ifndef MULTIAGENTCIRCLEMAP_VISUALIZATION_H
#define MULTIAGENTCIRCLEMAP_VISUALIZATION_H

#include <MultiAgentCircleMap/CircleVec.h>
class Visualization {
    CircleVec global_circles_;
    int robot_index_;

public:
    Visualization(CircleVec circle_vec, int robot_index): global_circles_(circle_vec), robot_index_(robot_index){}

    Eigen::Vector2d  calculatePixelCoordinates(Eigen::Vector3d global_position_centre, int robot_index);
    void showLandmarksOnFullImage();
    void vec2eigenmat3d(std::vector<std::vector<double> > vec, Eigen::Matrix3d& mat);
    void vec2eigenmat4d(std::vector<std::vector<double> > vec, Eigen::Matrix4d& mat);




};


#endif //MULTIAGENTCIRCLEMAP_VISUALIZATION_H
