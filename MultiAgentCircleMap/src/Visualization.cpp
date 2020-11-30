//
// Created by alg on 27/11/20.
//

#include "MultiAgentCircleMap/Visualization.h"
#include <opencv-3.3.1-dev/opencv2/core/matx.hpp>
#include <opencv-3.3.1-dev/opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

template <class T>
void showVar(T obj, std::string name){
    std::cout<<"\n"<<name<<": \n"<<obj;
}

void Visualization::vec2eigenmat3d(std::vector<std::vector<double> > vec, Eigen::Matrix3d& mat){
    for(int i =0; i< vec.size(); i++){
        for(int j =0; j< vec[i].size(); j++)
            mat(i,j) = vec[i][j];
    }
}

void Visualization::vec2eigenmat4d(std::vector<std::vector<double> > vec, Eigen::Matrix4d& mat){
    for(int i =0; i< vec.size(); i++){
        for(int j =0; j< vec[i].size(); j++)
            mat(i,j) = vec[i][j];
    }
}


Eigen::Vector2d Visualization::calculatePixelCoordinates(Eigen::Vector3d global_position_centre, int robot_index) {
    std::string filename = "/home/alg/RoverLocalization/rover_localization_ws/src/MultiAgentCircleMap/MultiAgentCircleMap/config/constants_iris" + std::to_string(robot_index) + ".yaml";
    YAML::Node config = YAML::LoadFile(filename);
    Eigen::Matrix3d K_MATRIX;
    std::vector<std::vector<double> > k_matrix_vec = config["k_matrix"].as<std::vector<std::vector<double>> >();
    vec2eigenmat3d(k_matrix_vec, K_MATRIX);
    Eigen::Vector3d px_3; //pixel values with 3 coordinates [u`, v`, w`] where final pixel values are given by u`/w`, v`/w`
    Eigen::Vector3d cTp;
    Eigen::Matrix4d rHc, rHc0, c0Hc,  wHr, wHp, cHp;

    wHp << 1, 0, 0, global_position_centre[0],
            0, 1, 0, global_position_centre[1],
            0, 0, 1, global_position_centre[2],
            0, 0, 0, 1;

    //showVar(wHp, "wHp");

    //rHc0
    std::vector<std::vector<double> > rHc0_vec = config["rHc0"].as<std::vector<std::vector<double>> >();
    vec2eigenmat4d(rHc0_vec, rHc0);
    //c0Hc
    std::vector<std::vector<double> > c0Hc_vec = config["c0Hc"].as<std::vector<std::vector<double>> >();
    vec2eigenmat4d(c0Hc_vec, c0Hc);
    rHc = rHc0*c0Hc;
    //robot position from where the big image was taken
    Eigen::Quaterniond big_image_robot_quat(0.998, -0.0019, 0.0013, 0.05);
    Eigen::Matrix3d rot(big_image_robot_quat);
    wHr << rot(0,0), rot(0,1), rot(0,2), 0,
           rot(1,0),rot(1,1), rot(1,2), -0.025,
           rot(2,0), rot(2,1), rot(2,2), 14.15,
            0, 0, 0, 1;

    //calculate cTp using
    cHp = (rHc.inverse())*(wHr.inverse())*wHp;
    cTp << cHp(0,3), cHp(1,3), cHp(2,3);
    //showVar(cTp, "cTp");

    px_3  =  K_MATRIX*cTp;
    Eigen::Vector2d px_2;
    px_2 << px_3[0]/px_3[2],  px_3[1]/px_3[2];
    return px_2;

}

void Visualization::showLandmarksOnFullImage() {

    //get the large image and note its position.. use the positions of the global_circle_vec to show the position of the circles it has detected
    cv::Mat img = cv::imread("/home/alg/RoverLocalization/rover_localization_ws/src/MultiAgentCircleMap/MultiAgentCircleMap/config/imagec.png");
    std::cout<<"Number of circles: "<<global_circles_.circle_vec_.size()<<std::endl;

    for(int i =0; i< this->global_circles_.circle_vec_.size(); i++){
        Eigen::Vector2d  px = calculatePixelCoordinates(this->global_circles_.circle_vec_[i].global_position_, robot_index_);

        cv::putText(img, std::to_string(this->global_circles_.circle_vec_[i].index_),
                    cv::Point(px[0], px[1]),
                    cv::FONT_HERSHEY_COMPLEX_SMALL,
                    0.5, cv::Scalar(255,255,255), 1, CV_AA );


    }

    cv::namedWindow("WithLandmarkIDs", CV_WINDOW_NORMAL);
    cv::imshow("WithLandmarkIDs",img);
    cv::waitKey(1);

}


