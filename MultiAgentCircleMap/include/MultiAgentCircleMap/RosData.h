#ifndef SRC_ROSDATA_H
#define SRC_ROSDATA_H
//#include <sensor_msgs/Imu.h>
//#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Geometry>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include "Image.h"
#include "../../../../../devel/include/MultiAgentCircleMap/CircleArray.h"
//#include "ArUcoMarker.h"

namespace MultiAgentCircleMap{
/**
 * Contains relevant data from ROS and makes sure the data is latest //TODO: call the resetBools() function at the appropriate location to make sure the data is latest.
 * **/

struct RosData {
public:

    /** ROS data variables */
    Eigen::Vector3d angular_velocity_;
    Eigen::Vector3d linear_velocity_;
    sensor_msgs::Image ros_image_;
    geometry_msgs::PoseStamped robot_pose_;
    Image image_;
    MultiAgentCircleMap::CircleArray global_circle0_array_; //global circles from other robots
    MultiAgentCircleMap::CircleArray global_circle1_array_; //global circles from other robots
    MultiAgentCircleMap::CircleArray global_circle2_array_; //global circles from other robots

    /** Bools to make sure new data is used*/
    bool new_angular_velocity_;
    bool new_linear_velocity_;
    bool new_image_;
    bool new_robot_pose_;
    bool new_global_circle0_;
    bool new_global_circle1_;
    bool new_global_circle2_;



    //ArUco Marker
    //ArUcoMarker aruco_marker_;  //aruco marker in the current image obtained from ros

public:
    void setAngularVelocity(sensor_msgs::Imu imu_message){
        this->angular_velocity_ << imu_message.angular_velocity.x, imu_message.angular_velocity.y, imu_message.angular_velocity.z;
    }

    void setLinearVelocity(geometry_msgs::PoseStamped pose_message){ //TODO: Change the type of message depending on what type of odometry topic is used. maybe overload the function
        this->linear_velocity_ << pose_message.pose.position.x, pose_message.pose.position.y, pose_message.pose.position.z;

    }

    void setROSImage(sensor_msgs::Image image){
        this->ros_image_ = image;
        //new_image_ = true;
    }

    void setImage(Image image){
        this->image_ = image;
    }

    void setRobotPose(geometry_msgs::PoseStamped robot_pose) {
        robot_pose_ = robot_pose;
        //new_robot_pose_ = true;
    }

    void resetBools(){
        this->new_angular_velocity_ = false;
        this->new_linear_velocity_ = false;
        this->new_image_ = false;
        this->new_robot_pose_ = false;
        this->new_global_circle0_ = false;
        this->new_global_circle1_ = false;
        this->new_global_circle2_ = false;
    }

    void setBoolNewAngularVelocity(bool is_new_angular_velocity){
        this->new_linear_velocity_ = is_new_angular_velocity;
    }

    void setBoolNewLinearVelocity(bool is_new_linear_velocity){
        this->new_linear_velocity_ = is_new_linear_velocity;
    }

    void setBoolNewImage(bool is_new_image){
        this->new_image_ = is_new_image;
    }

    bool isNewLinearVelocity(){
        return this->new_linear_velocity_;
    }

    bool isNewAngularVelocity(){
        return this->new_angular_velocity_;
    }

    bool isNewImage(){
        return this->new_image_;
    }

    bool isNewRobotPose(){
        return this->new_robot_pose_;
    }


};


}


#endif //SRC_ROSDATA_H
