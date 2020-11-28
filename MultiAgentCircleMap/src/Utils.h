//
// Created by alg on 26/11/20.
//

#ifndef MULTIAGENTCIRCLEMAP_UTILS_H
#define MULTIAGENTCIRCLEMAP_UTILS_H
#include "MultiAgentCircleMap/CircleVec.h"
#include <opencv-3.3.1-dev/opencv2/core/matx.hpp>
#include <opencv-3.3.1-dev/opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/yaml.h>





/*Eigen::Vector2d  calculatePixelCoordinates(Eigen::Vector3d global_position_centre, int robot_index){
    std::string filename = "/home/alg/RoverLocalization/rover_localization_ws/src/MultiAgentCircleMap/MultiAgentCircleMap/config/constants_iris" + std::to_string(robot_index) + ".yaml";
    YAML::Node config = YAML::LoadFile(filename);
    Eigen::Matrix3d K_MATRIX;
    std::vector<std::vector<double> > k_matrix_vec = config["k_matrix"].as<std::vector<std::vector<double>> >();
    vec2eigenmat3d(k_matrix_vec, K_MATRIX);
    Eigen::Vector3d px_3; //pixel values with 3 coordinates [u`, v`, w`] where final pixel values are given by u`/w`, v`/w`
    Eigen::Vector3d cTp;
    Eigen::Matrix4d rHc, rHc0, c0Hc,  wHr, wHp, cHp;
    std::vector<std::vector<double> > rHc0_vec = config["rHc0"].as<std::vector<std::vector<double>> >();
    vec2eigenmat4d(rHc0_vec, rHc0);
    //calculate cTp using
    cHp = (rHc.inverse())*(wHr.inverse())*wHp;
    cTp << cHp(0,3), cHp(1,3), cHp(2,3);

    px_3  =  K_MATRIX*cTp;
}*/

/*void showLandmarksOnFullImage(CircleVec global_circles, int robot_index){
    //get the large image and note its position.. use the positions of the global_circle_vec to show the position of the circles it has detected
    cv::Mat img = cv::imread("/home/alg/RoverLocalization/rover_localization_ws/src/MultiAgentCircleMap/MultiAgentCircleMap/config/circles15mheight.png");

    for(int i =0; i< global_circles.circle_vec_.size(); i++){
        Eigen::Vector2d  px = calculatePixelCoordinates(global_circles.circle_vec_[i].global_position_, robot_index);

        cv::putText(img, std::to_string(global_circles.circle_vec_[i].index_),
                cv::Point(px[0], px[1]),
                cv::FONT_HERSHEY_COMPLEX_SMALL,
                0.5, cv::Scalar(255,255,255), 1, CV_AA );


    }

    cv::namedWindow("WithLandmarkIDs", CV_WINDOW_NORMAL);
    cv::imshow("WithLandmarkIDs",img);
    cv::waitKey();



}*/

#endif //MULTIAGENTCIRCLEMAP_UTILS_H
