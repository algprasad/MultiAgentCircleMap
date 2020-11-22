//
// Created by alg on 21/11/20.
//

#ifndef MULTIAGENTCIRCLEMAP_IMAGE_H
#define MULTIAGENTCIRCLEMAP_IMAGE_H

#include <sensor_msgs/Image.h>
#include "opencv2/opencv.hpp"
#include "CircleVec.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

class Image {
    cv::Mat image_;
    cv::Mat image_detected_circles_;
    CircleVec circle_vec_;  //vector of circles associated with the image

public:
    //Constructor to construct image with cv::Mat
    Image(cv::Mat image): image_(image){ detectCircles(); }

    //Default constructor
    Image(){}


public:
    /* Detect circles in the current image*/
    void detectCircles();


//Constructor to construct image with sensor_msgs::Image
Image(sensor_msgs::Image msg){
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    image_ = cv_ptr->image;
    detectCircles();
}

};


#endif //MULTIAGENTCIRCLEMAP_IMAGE_H
