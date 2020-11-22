//
// Created by alg on 21/11/20.
//

#include "MultiAgentCircleMap/Image.h"
#include "opencv2/imgproc.hpp"


void Image::detectCircles() {

    cv::Mat gray;
    cv::cvtColor(image_, gray, cv::COLOR_BGR2GRAY);
    medianBlur(gray, gray, 5);
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1,
                     gray.rows/16,  // change this value to detect circles with different distances to each other
                     100, 30, 1, 30 // change the last two parameters
            // (min_radius & max_radius) to detect larger circles
    );
    CircleVec circle_vec(circles);

}
