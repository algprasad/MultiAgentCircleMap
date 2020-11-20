#include "MultiAgentCircleMap/RosHandle.hpp"
#include <geometry_msgs/PoseStamped.h>

// STD
#include <string>

namespace MultiAgentCircleMap {

RosHandle::RosHandle(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle), rate_(ros_rate_hz)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  imu_subscriber_ = nodeHandle_.subscribe(imu_subscriber_topic_, 1,
                                          &RosHandle::imuCallback, this);

  image_subscriber_ = nodeHandle_.subscribe(image_subscriber_topic_, 1,
                                            &RosHandle::imageCallback, this);

  odometry_subscriber_ = nodeHandle_.subscribe(odometry_subscriber_topic_, 1, &RosHandle::odometryCallback, this );

  serviceServer_ = nodeHandle_.advertiseService("future_addition",
                                                &RosHandle::serviceCallback, this);
  ROS_INFO("Successfully launched node.");
}

RosHandle::~RosHandle()
{
}

bool RosHandle::readParameters()
{
  if (!nodeHandle_.getParam("ros_rate", ros_rate_hz)) return false;
  if (!nodeHandle_.getParam("sub_imu_topic", imu_subscriber_topic_)) return false;
  if (!nodeHandle_.getParam("sub_image_topic", image_subscriber_topic_)) return false;
  if (!nodeHandle_.getParam("sub_odometry_topic", odometry_subscriber_topic_)) return false;

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

void RosHandle::odometryCallback(const geometry_msgs::PoseStamped &message) {
    ros_data_.setLinearVelocity(message);
    ros_data_.setBoolNewLinearVelocity(true);

}

void RosHandle::imageCallback(const sensor_msgs::Image &message) {
    ros_data_.setImage(message);
    ros_data_.setBoolNewImage(true);

}

} /* namespace */
