#pragma once


// ROS
#include <ros/ros.h>
//#include <sensor_msgs/Temperature.h>
//#include <sensor_msgs/Imu.h>
//#include <sensor_msgs/Image.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/PoseStamped.h>
#include "RosData.h"
#include "../../../../devel/include/MultiAgentCircleMap/CircleMsg.h"
#include "../../../../devel/include/MultiAgentCircleMap/CircleArray.h"


namespace MultiAgentCircleMap {

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class RosHandle
{
private:
    //! ROS node handle.
    ros::NodeHandle& node_handle_;

    //! ROS topic subscriber.
    ros::Subscriber imu_subscriber_;
    ros::Subscriber odometry_subscriber_;
    ros::Subscriber image_subscriber_;
    ros::Subscriber sub_global_circles_0_;
    ros::Subscriber sub_global_circles_1_;
    ros::Subscriber sub_global_circles_2_;
    ros::Subscriber sub_global_circles_3_;
    ros::Subscriber sub_global_circles_4_;


    //! ROS Publisher
    image_transport::Publisher pub_image_detected_circles_ ;
    ros::Publisher pub_global_circles_;



    //! ROS topic name to subscribe to.
    std::string imu_subscriber_topic_;
    std::string robot_pose_subscriber_topic_;
    std::string image_subscriber_topic_;
    std::string global_circles_subscriber_topic_0_;
    std::string global_circles_subscriber_topic_1_;
    std::string global_circles_subscriber_topic_2_;
    std::string global_circles_subscriber_topic_3_;
    std::string global_circles_subscriber_topic_4_;


    //! ROS topic names to publish to
    std::string detected_circles_image_publisher_topic_;
    std::string global_circles_publisher_topic_;


    //! ROS service server.
    ros::ServiceServer serviceServer_;

public:
    /**
    * Robot index to identify which robot is being referenced
    * */
    int robot_index_;

    //ROSData object to which the values are passed
    RosData ros_data_;
    double ros_rate_hz_;
    ros::Rate rate_;


    //Other constants
    //Hungarian Algo based constants
    double threshold_pixel_distance_;
    double COMMUNICATION_THRESHOLD_;


   /*!
    * Constructor.
    * @param node_handle the ROS node handle.
    */
   RosHandle(ros::NodeHandle& node_handle, int robot_index);


    /*!
     * Destructor.
     */
    virtual ~RosHandle();



    void pubDetectedCirclesImage(cv::Mat img);

    //function to publish circles for others to see and merge
    void pubGlobalDetectedCircles(CircleVec global_circle_vec);



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
  void globalCircles0Callback(const MultiAgentCircleMap::CircleArray& msg);
  void globalCircles1Callback(const MultiAgentCircleMap::CircleArray& msg);
  void globalCircles2Callback(const MultiAgentCircleMap::CircleArray& msg);
  void globalCircles3Callback(const MultiAgentCircleMap::CircleArray& msg);
  void globalCircles4Callback(const MultiAgentCircleMap::CircleArray& msg);
  double distanceFromCurrentRobot(Eigen::Vector3d other_robot_position);




    /*!
     * ROS service server callback.
     * @param request the request of the service.
     * @param response the provided response.
     * @return true if successful, false otherwise.
     */
  bool serviceCallback(std_srvs::Trigger::Request& request,
                       std_srvs::Trigger::Response& response);







};

} /* namespace */
