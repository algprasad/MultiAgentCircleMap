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
    cv::Mat cv_image_; //main raw image
    cv::Mat cv_image_detected_circles_; //image with circles and their IDs marked
    geometry_msgs::PoseStamped robot_pose_; //position of the robot from where the image was acquired
public:

    CircleVec circle_vec_;  //vector of circles associated with the image
    bool used_pixels_;  //bool to determine if pixel coordinates or position coordinates will used for data association
    int robot_index_;

public:
    unsigned int size_; //size of circle_vec
    
    //Constructor to construct image with cv::Mat
    Image(cv::Mat image): cv_image_(image), size_(0){ detectCircles(); }

    //Default constructor
    Image(){}


public:
    /** Detect circles in the current image and add to circle_vec_ **/
    void detectCircles();

    /** Method to assign position coordinates to circles  */
    void assignPositionCoordinates2Circles();

    /** Constructor to construct image with sensor_msgs::Image*/
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
        cv_image_ = cv_ptr->image;
        used_pixels_ = false;
        this->size_ = 0;
        detectCircles();
    }

    /** Constructor to construct image with sensor_msgs::Image and also the robot_pose*/
    Image(sensor_msgs::Image msg,
            geometry_msgs::PoseStamped robot_pose,
            bool used_pixels, int robot_index):
            robot_pose_(robot_pose),
            used_pixels_(false),
            robot_index_(robot_index){
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
        cv_image_ = cv_ptr->image;
        this->size_ = 0;
        detectCircles();
    }
    unsigned int getSize();

    void writeLandmarkID();
    cv::Mat getImageWithDetectedCircles(){
        return cv_image_detected_circles_;
    }



};


#endif //MULTIAGENTCIRCLEMAP_IMAGE_H
