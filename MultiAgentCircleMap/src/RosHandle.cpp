#include "MultiAgentCircleMap/RosHandle.hpp"
#include <geometry_msgs/PoseStamped.h>

// STD
#include <string>
#include <MultiAgentCircleMap/Image.h>
#include <fstream>
#include "yaml-cpp/yaml.h"


namespace MultiAgentCircleMap {

RosHandle::RosHandle(ros::NodeHandle& node_handle, int robot_index)
    : node_handle_(node_handle), rate_(20), robot_index_(robot_index) //assigning temporary value which will be overwritten by the rosparam
{
  if (!readParametersYAML()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  rate_ = ros_rate_hz_;
  imu_subscriber_ = node_handle_.subscribe(imu_subscriber_topic_, 1,
                                           &RosHandle::imuCallback, this);

  image_subscriber_ = node_handle_.subscribe(image_subscriber_topic_, 1,
                                             &RosHandle::imageCallback, this);

  /** Subscribe to other robots' local maps*/

  sub_global_circles_0_ = node_handle_.subscribe(global_circles_subscriber_topic_0_, 10,
                                            &RosHandle::globalCircles0Callback, this);

  sub_global_circles_1_ = node_handle_.subscribe(global_circles_subscriber_topic_1_, 10,
                                                   &RosHandle::globalCircles1Callback, this);
  sub_global_circles_2_ = node_handle_.subscribe(global_circles_subscriber_topic_2_, 10,
                                                   &RosHandle::globalCircles2Callback, this);


  odometry_subscriber_ = node_handle_.subscribe(robot_pose_subscriber_topic_, 1, &RosHandle::PoseCallback, this );

  image_transport::ImageTransport it(node_handle_);
    pub_image_detected_circles_ = it.advertise(detected_circles_image_publisher_topic_, 1);

  pub_global_circles_ = node_handle_.advertise<MultiAgentCircleMap::CircleArray>(global_circles_publisher_topic_, 10);

  serviceServer_ = node_handle_.advertiseService("future_addition",
                                                 &RosHandle::serviceCallback, this);
  ros_data_.resetBools();


  ROS_INFO("Successfully launched node.");
}

RosHandle::~RosHandle()
{
}

//DEPRECATED Used readParametersYAML() instead
bool RosHandle::readParameters()
{
  if (!node_handle_.getParam("ros_rate", ros_rate_hz_)) return false;
  if (!node_handle_.getParam("sub_imu_topic", imu_subscriber_topic_)) return false;
  if (!node_handle_.getParam("sub_image_topic", image_subscriber_topic_)) return false;
  if (!node_handle_.getParam("robot_pose_topic", robot_pose_subscriber_topic_)) return false;
  if (!node_handle_.getParam("pub_detected_circles_image", detected_circles_image_publisher_topic_)) return false;
  if (!node_handle_.getParam("threshold_pixel_distance", threshold_pixel_distance_)) return false;

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
    ros_data_.new_image_ = true;
    //Make the image object and assign the robot pose to the image object
    Image image(message, ros_data_.robot_pose_, false, robot_index_);
    if(image.circle_vec_.circle_vec_.size() > 0) ros_data_.setImage(image); //to make sure that when the circles are not there unnecessary calculations are not done

}

void RosHandle::pubDetectedCirclesImage(cv::Mat img) {
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    pub_image_detected_circles_.publish(msg);
}

bool RosHandle::readParametersYAML() {
    std::string filename = "/home/alg/RoverLocalization/rover_localization_ws/src/MultiAgentCircleMap/MultiAgentCircleMap/config/constants_iris" + std::to_string(robot_index_) + ".yaml";
    YAML::Node config = YAML::LoadFile(filename);
    ros_rate_hz_ = config["ros_rate"].as<double_t >();
    imu_subscriber_topic_ = config["sub_imu_topic"].as<std::string>();
    image_subscriber_topic_ = config["sub_image_topic"].as<std::string>();
    robot_pose_subscriber_topic_ = config["robot_pose_topic"].as<std::string>();
    detected_circles_image_publisher_topic_ = config["pub_detected_circles_image"].as<std::string>();
    threshold_pixel_distance_ = config["threshold_pixel_distance"].as<double_t >();
    global_circles_publisher_topic_ = config["global_circles_publisher_topic"].as<std::string>();
    global_circles_subscriber_topic_0_ = config["sub_global_circles_0_topic"].as<std::string>();
    global_circles_subscriber_topic_1_ = config["sub_global_circles_1_topic"].as<std::string>();
    global_circles_subscriber_topic_2_ = config["sub_global_circles_2_topic"].as<std::string>();
    COMMUNICATION_THRESHOLD_ = config["communication_threshold"].as<double_t >();


    return true;  //FIXME the yaml should return boolean to check if the parameters were read and then readParametersYAML can use that.
}

void RosHandle::pubGlobalDetectedCircles(CircleVec global_circle_vec) {
    MultiAgentCircleMap::CircleArray circle_array;
    for(int i =0; i< global_circle_vec.circle_vec_.size(); i++){
        MultiAgentCircleMap::CircleMsg temp_circle_msg;
        temp_circle_msg.centre_x = global_circle_vec.circle_vec_[i].global_position_[0];
        temp_circle_msg.centre_y = global_circle_vec.circle_vec_[i].global_position_[1];
        temp_circle_msg.centre_z = global_circle_vec.circle_vec_[i].global_position_[2];
        temp_circle_msg.radius = global_circle_vec.circle_vec_[i].radius_;
        temp_circle_msg.index = global_circle_vec.circle_vec_[i].index_;

        circle_array.circle_vec.push_back(temp_circle_msg);
    }

    //broadcast the position of the robot too to check the distance
    circle_array.robot_x = ros_data_.robot_pose_.pose.position.x;
    circle_array.robot_y = ros_data_.robot_pose_.pose.position.y;
    circle_array.robot_z = ros_data_.robot_pose_.pose.position.z;


    pub_global_circles_.publish(circle_array);
}

double RosHandle::distanceFromCurrentRobot(Eigen::Vector3d other_robot_position){
    Eigen::Vector3d current_robot_position(ros_data_.robot_pose_.pose.position.x,
                                           ros_data_.robot_pose_.pose.position.y,
                                           ros_data_.robot_pose_.pose.position.z);

    Eigen::Vector3d temp_diff_vec  = current_robot_position  - other_robot_position;
    double distance = temp_diff_vec.norm();
    return distance;
}

//FIXME confirm if you can use the same callback function for all the three
void RosHandle::globalCircles0Callback(const MultiAgentCircleMap::CircleArray& msg) {
    ros_data_.global_circle0_array_ = msg;
    //check distance from the current robot and set new_global_circle_x to false if its greater than the communication threshold
    if(distanceFromCurrentRobot(Eigen::Vector3d(ros_data_.global_circle0_array_.robot_x,
                                                ros_data_.global_circle0_array_.robot_y,
                                                ros_data_.global_circle0_array_.robot_z)) < COMMUNICATION_THRESHOLD_) ros_data_.new_global_circle0_ = true;
}

void RosHandle::globalCircles1Callback(const MultiAgentCircleMap::CircleArray& msg) {
    ros_data_.global_circle1_array_ = msg;
    //check distance from the current robot and set new_global_circle_x to false if its greater than the communication threshold
    if(distanceFromCurrentRobot(Eigen::Vector3d(ros_data_.global_circle1_array_.robot_x,
                                                ros_data_.global_circle1_array_.robot_y,
                                                ros_data_.global_circle1_array_.robot_z)) < COMMUNICATION_THRESHOLD_) ros_data_.new_global_circle1_ = true;

}

void RosHandle::globalCircles2Callback(const MultiAgentCircleMap::CircleArray& msg) {

    ros_data_.global_circle2_array_ = msg;
    //check distance from the current robot and set new_global_circle_x to false if its greater than the communication threshold
    if(distanceFromCurrentRobot(Eigen::Vector3d(ros_data_.global_circle2_array_.robot_x,
                                                ros_data_.global_circle2_array_.robot_y,
                                                ros_data_.global_circle2_array_.robot_z)) < COMMUNICATION_THRESHOLD_) ros_data_.new_global_circle2_ = true;
}

void RosHandle::globalCircles3Callback(const MultiAgentCircleMap::CircleArray& msg) {

        ros_data_.global_circle3_array_ = msg;
        //check distance from the current robot and set new_global_circle_x to false if its greater than the communication threshold
        if(distanceFromCurrentRobot(Eigen::Vector3d(ros_data_.global_circle3_array_.robot_x,
                                                    ros_data_.global_circle3_array_.robot_y,
                                                    ros_data_.global_circle3_array_.robot_z)) < COMMUNICATION_THRESHOLD_) ros_data_.new_global_circle3_ = true;
}
void RosHandle::globalCircles4Callback(const MultiAgentCircleMap::CircleArray& msg) {

        ros_data_.global_circle4_array_ = msg;
        //check distance from the current robot and set new_global_circle_x to false if its greater than the communication threshold
        if(distanceFromCurrentRobot(Eigen::Vector3d(ros_data_.global_circle4_array_.robot_x,
                                                    ros_data_.global_circle4_array_.robot_y,
                                                    ros_data_.global_circle4_array_.robot_z)) < COMMUNICATION_THRESHOLD_) ros_data_.new_global_circle4_ = true;
}

} /* namespace */
