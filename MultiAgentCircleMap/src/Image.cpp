//
// Created by alg on 21/11/20.
//

#include "MultiAgentCircleMap/Image.h"
#include "opencv2/imgproc.hpp"


void Image::detectCircles() {

    cv::Mat gray;
    cv::cvtColor(cv_image_, gray, cv::COLOR_BGR2GRAY);
    medianBlur(gray, gray, 5);
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1,
                     gray.rows/16,  // change this value to detect circles with different distances to each other
                     100, 30, 1, 30 // change the last two parameters
            // (min_radius & max_radius) to detect larger circles
    );
    CircleVec circle_vec(circles);
    this->circle_vec_ = circle_vec;
    this->size_ = circles.size();
    cv_image_detected_circles_ = cv_image_; //so that the image_detected_circles has the circles highlighted



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
