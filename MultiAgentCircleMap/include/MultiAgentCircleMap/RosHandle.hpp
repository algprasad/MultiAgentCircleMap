#pragma once


// ROS
#include <ros/ros.h>
//#include <sensor_msgs/Temperature.h>
//#include <sensor_msgs/Imu.h>
//#include <sensor_msgs/Image.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/PoseStamped.h>
#include "RosData.h"

namespace MultiAgentCircleMap {

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class RosHandle
{
 public:
  /**
   * Robot index to identify which robot is being referenced
   * */
   int robot_index_;


  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  RosHandle(ros::NodeHandle& nodeHandle, int robot_index);


  /*!
   * Destructor.
   */
  virtual ~RosHandle();

    //ROSData object to which the values are passed
    RosData ros_data_;
    double ros_rate_hz_;
    ros::Rate rate_;


    //Other constants
    //Hungarian Algo based constants
    double threshold_pixel_distance_;

    void pubDetectedCircles(cv::Mat img);

private:

  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParametersYAML();


  /*!
   * IMU topic callback method.
   * @param for imu messages given by pixhawk.
   */
  void imuCallback(const sensor_msgs::Imu& message);

  /*!
   * IMU topic callback method.
   * @param for imu messages given by pixhawk.
   */
  void imageCallback(const sensor_msgs::Image& message);

  /*!
   * IMU topic callback method.
   * @param odometry message given by optical flow
   */
  void PoseCallback(const geometry_msgs::PoseStamped& message);


    /*!
     * ROS service server callback.
     * @param request the request of the service.
     * @param response the provided response.
     * @return true if successful, false otherwise.
     */
  bool serviceCallback(std_srvs::Trigger::Request& request,
                       std_srvs::Trigger::Response& response);

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! ROS topic subscriber.
  ros::Subscriber imu_subscriber_;
  ros::Subscriber odometry_subscriber_;
  ros::Subscriber image_subscriber_;

  //! ROS Publisher
  image_transport::Publisher pub_detected_circles_ ;


  //! ROS topic name to subscribe to.
  std::string imu_subscriber_topic_;
  std::string robot_pose_subscriber_topic_;
  std::string image_subscriber_topic_;

  //! ROS topic names to publish to
  std::string detected_circles_publisher_topic_;

  //! ROS service server.
  ros::ServiceServer serviceServer_;


};

} /* namespace */
