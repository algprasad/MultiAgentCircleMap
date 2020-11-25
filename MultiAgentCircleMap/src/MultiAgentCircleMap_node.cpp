#include <ros/ros.h>
#include <MultiAgentCircleMap/Graph.h>
#include <MultiAgentCircleMap/CircleVec.h>
#include <MultiAgentCircleMap/SingleRobotMapper.h>
#include "MultiAgentCircleMap/RosHandle.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "MultiAgentCircleMap");
  ros::NodeHandle node_handle("~");

  MultiAgentCircleMap::RosHandle ros_handle(node_handle);
  SingleRobotMapper single_robot_mapper;  //ros_handle_ has all the latest information from ros in ros_data_ and gets updated every cycle

  while(ros::ok()){

      if(ros_handle.ros_data_.isNewImage()) single_robot_mapper.updateMap(ros_handle);
      ros_handle.ros_data_.resetBools();
      //single_robot_mapper.publishImagewithIDs();
      ros::spinOnce();

  }

  return 0;
}
