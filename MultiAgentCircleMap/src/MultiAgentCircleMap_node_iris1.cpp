#include <ros/ros.h>
#include <MultiAgentCircleMap/Graph.h>
#include <MultiAgentCircleMap/CircleVec.h>
#include <MultiAgentCircleMap/SingleRobotMapper.h>
#include <MultiAgentCircleMap/Visualization.h>
#include "MultiAgentCircleMap/RosHandle.hpp"
#include "MultiAgentCircleMap/MultiRobotMapper.h"
#include "Utils.h"
const int robot_index = 1; //Change this for the subsequent robots (i.e. #1,  #2)
const int NUM_ROBOTS = 3;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "MultiAgentCircleMap_Iris1");
  ros::NodeHandle node_handle("~");

  MultiAgentCircleMap::RosHandle ros_handle(node_handle, robot_index);
  //build local map
  SingleRobotMapper single_robot_mapper;  //ros_handle_ has all the latest information from ros in ros_data_ and gets updated every cycle

  //combine maps from other robots
  MultiRobotMapper multi_robot_mapper(robot_index);
  while(ros::ok()){

      if(ros_handle.ros_data_.isNewImage() && ros_handle.ros_data_.isNewRobotPose())
          single_robot_mapper.updateMap(ros_handle);
      //publish the circles
      ros_handle.pubGlobalDetectedCircles(single_robot_mapper.getGlobalCircles());

      //TODO: change this for every robot node
      if(ros_handle.ros_data_.isNewRobotPose() && ((ros_handle.ros_data_.new_global_circle0_ ) || (ros_handle.ros_data_.new_global_circle2_))){
          multi_robot_mapper.updateMap(ros_handle, single_robot_mapper);
          std::cout<<"\nOne of the robots is close \n" ;

      }


      ros_handle.ros_data_.resetBools();

      // Display all the detections with their indices on an image plane with cv::window
      Visualization visualizer(single_robot_mapper.getGlobalCircles(), ros_handle.robot_index_);
      visualizer.showLandmarksOnFullImage();




      ros::spinOnce();
      ros_handle.rate_.sleep();

  }

  return 0;
}
