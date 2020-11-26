//
// Created by alg on 21/11/20.
//

#include <geometry_msgs/PoseStamped.h>
#include "MultiAgentCircleMap/Image.h"
#include "opencv2/imgproc.hpp"


void Image::detectCircles() {

    cv::Mat gray;
    cv::cvtColor(cv_image_, gray, cv::COLOR_BGR2GRAY);
    medianBlur(gray, gray, 5);
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1,
                     gray.rows/16,  // change this value to detect circles with different distances to each other
                     100, 30, 1, 50 // change the last two parameters
            // (min_radius & max_radius) to detect larger circles
    );
    CircleVec circle_vec(circles);
    this->circle_vec_ = circle_vec;
    this->size_ = circles.size();
    cv_image_detected_circles_ = cv_image_; //so that the image_detected_circles has the circles highlighted
    //if(!used_pixels_) assignPositionCoordinates2Circles();



}

void Image::writeLandmarkID() {

    for(int i =0; i< this->getSize(); i++){
        Eigen::Vector2d pixel = circle_vec_.circle_vec_[i].pixel_coords_.top();
        cv::putText(cv_image_detected_circles_, std::to_string(circle_vec_.circle_vec_[i].index_), cvPoint(pixel[0], pixel[1]), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cv::Scalar(255, 255, 255), 1, CV_AA );
    }

}

unsigned int Image::getSize() {
    return circle_vec_.circle_vec_.size();

}

void Image::assignPositionCoordinates2Circles() {
    Eigen::Matrix3d K_MATRIX;
    K_MATRIX << 238.35, 0, 200.5,
                0, 238.35, 200.5,
                0, 0, 1;
    double INITIAL_X = 0, INITIAL_Y = 1, INITIAL_Z = 5; //TODO: CHECK THESE VALUES

    for(int i =0; i< circle_vec_.circle_vec_.size(); i++){

        //coordinates of robot in world frame //FIXME: Assumes the orientation is fixed which is not the case.
        Eigen::Matrix4d wHr, r0Hr, wHr0; //wHr0 is the initial position of the robot

        /***
         * Because we only get local position, and start from 0,0,0, the initial position is incorporated separately
         * */
        wHr0 << 1, 0, 0, INITIAL_X,
                0, 1, 0, INITIAL_Y,
                0, 0, 1, INITIAL_Z,
                0, 0, 0, 1;

        //TODO: get the orientation too
        Eigen::Quaterniond robot_orientation_eigen_quat;
        robot_orientation_eigen_quat.x() = robot_pose_.pose.orientation.x;
        robot_orientation_eigen_quat.y() = robot_pose_.pose.orientation.y;
        robot_orientation_eigen_quat.z() = robot_pose_.pose.orientation.z;
        robot_orientation_eigen_quat.w() = robot_pose_.pose.orientation.w;

        Eigen::Matrix3d rmat(robot_orientation_eigen_quat);

        r0Hr << rmat(0,0), rmat(0,1), rmat(0,2), robot_pose_.pose.position.x,
                rmat(1,0), rmat(1,1), rmat(1,2), robot_pose_.pose.position.y,
                rmat(2,0), rmat(2,1), rmat(2,2), robot_pose_.pose.position.z,
                0, 0, 0, 1;

        wHr = wHr0*r0Hr;
        double z = wHr(2,3);
        Eigen::Vector3d cTp; //position of pixel coordinate in the frame of camera
        Eigen::Vector3d pixel_coordinates;
        pixel_coordinates << this->circle_vec_.circle_vec_[i].pixel_coords_.top(), 1;  //u, v, 1
        //coordinates of the centre in camera frame
        cTp = K_MATRIX.inverse() * z * pixel_coordinates;

        //making it homogenous
        Eigen::Matrix4d cHp;
        cHp << 1, 0, 0, cTp[0],
                0, 1, 0, cTp[1],
                0, 0, 1, cTp[2],
                0, 0, 0, 1;

        //need the camera in robot frame
        Eigen::Matrix4d rHc; //camera pointing downwards
        rHc << 0, 0, 1, 0.2,
                0, 1, 0, 0,
                -1, 0, 0, 0.03,
                0, 0, 0, 1;

        //coordinates of the centre in global frame
        Eigen::Matrix4d wHp;
        wHp  = wHr*rHc*cHp;
        Eigen::Vector3d wTp;
        wTp << wHp(0, 3), wHp(1,3), wHp(2,3);
        this->circle_vec_.circle_vec_[i].global_position_ = wTp;

    }


}