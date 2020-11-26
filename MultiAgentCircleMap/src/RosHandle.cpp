#include "MultiAgentCircleMap/RosHandle.hpp"
#include <geometry_msgs/PoseStamped.h>

// STD
#include <string>
#include <MultiAgentCircleMap/Image.h>
#include <fstream>
#include "yaml-cpp/yaml.h"


namespace MultiAgentCircleMap {

RosHandle::RosHandle(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle), rate_(20) //assigning temporary value which will be overwritten by the rosparam
{
  if (!readParametersYAML()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  rate_ = ros_rate_hz_;
  imu_subscriber_ = nodeHandle_.subscribe(imu_subscriber_topic_, 1,
                                          &RosHandle::imuCallback, this);

  image_subscriber_ = nodeHandle_.subscribe(image_subscriber_topic_, 1,
                                            &RosHandle::imageCallback, this);

  odometry_subscriber_ = nodeHandle_.subscribe(robot_pose_subscriber_topic_, 1, &RosHandle::PoseCallback, this );

  image_transport::ImageTransport it(nodeHandle_);
  pub_detected_circles_ = it.advertise(detected_circles_publisher_topic_, 1);

  serviceServer_ = nodeHandle_.advertiseService("future_addition",
                                                &RosHandle::serviceCallback, this);
  ros_data_.resetBools();
  ROS_INFO("Successfully launched node.");
}

RosHandle::~RosHandle()
{
}

bool RosHandle::readParameters()
{
  if (!nodeHandle_.getParam("ros_rate", ros_rate_hz_)) return false;
  if (!nodeHandle_.getParam("sub_imu_topic", imu_subscriber_topic_)) return false;
  if (!nodeHandle_.getParam("sub_image_topic", image_subscriber_topic_)) return false;
  if (!nodeHandle_.getParam("robot_pose_topic", robot_pose_subscriber_topic_)) return false;
  if (!nodeHandle_.getParam("pub_detected_circles_image", detected_circles_publisher_topic_)) return false;
  if (!nodeHandle_.getParam("threshold_pixel_distance", threshold_pixel_distance_)) return false;

  return true;
}

void RosHandle::imuCallback(const sensor_msgs::Imu& message)
{
  ros_data_.setAngularVelocity(message);
  ros_data_.setBoolNewAngularVelocity(true);
}

bool RosHandle::serviceCallback(std_srvs::Trigger::Request& request,
                                std_srvs::Trigger::Response& response)
{
  //response.success = true;
  //response.message = "The average is " + std::to_string(algorithm_.getAverage());
  return true;
}

void RosHandle::PoseCallback(const geometry_msgs::PoseStamped &message) {
    ros_data_.setRobotPose(message);
    ros_data_.new_robot_pose_ = true;

}

void RosHandle::imageCallback(const sensor_msgs::Image &message) {
    ros_data_.setROSImage(message);
    ros_data_.setBoolNewImage(true);
    //Make the image object and assign the robot pose to the image object
    Image image(message, ros_data_.robot_pose_); //
    //Image image(message);
    if(image.circle_vec_.circle_vec_.size() > 0) ros_data_.setImage(image); //to make sure that when the circles are not there unnecessary calculations are not done

}

void RosHandle::pubDetectedCircles(cv::Mat img) {
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    pub_detected_circles_.publish(msg);
}

bool RosHandle::readParametersYAML() {

    YAML::Node config = YAML::LoadFile("/home/alg/RoverLocalization/rover_localization_ws/src/MultiAgentCircleMap/MultiAgentCircleMap/config/default.yaml");
    ros_rate_hz_ = config["ros_rate"].as<double_t >();
    imu_subscriber_topic_ = config["sub_imu_topic"].as<std::string>();
    image_subscriber_topic_ = config["sub_image_topic"].as<std::string>();
    robot_pose_subscriber_topic_ = config["robot_pose_topic"].as<std::string>();
    detected_circles_publisher_topic_ = config["pub_detected_circles_image"].as<std::string>();
    threshold_pixel_distance_ = config["threshold_pixel_distance"].as<double_t >();

    return true;  //FIXME the yaml should return boolean to check if the parameters were read and then readParametersYAML can use that.
}


} /* namespace */
