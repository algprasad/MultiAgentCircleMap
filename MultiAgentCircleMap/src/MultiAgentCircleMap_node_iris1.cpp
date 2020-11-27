#include <ros/ros.h>
#include <MultiAgentCircleMap/Graph.h>
#include <MultiAgentCircleMap/CircleVec.h>
#include <MultiAgentCircleMap/SingleRobotMapper.h>
#include "MultiAgentCircleMap/RosHandle.hpp"
#include "MultiAgentCircleMap/MultiRobotMapper.h"

const int robot_index = 1; //Change this for the subsequent robots (i.e. #1,  #2)

int main(int argc, char** argv)
{
  ros::init(argc, argv, "MultiAgentCircleMap");
  ros::NodeHandle node_handle("~");

  MultiAgentCircleMap::RosHandle ros_handle(node_handle, robot_index);
  //build local map
  SingleRobotMapper single_robot_mapper;  //ros_handle_ has all the latest information from ros in ros_data_ and gets updated every cycle

  //combine maps from other robots
  //MultiRobotMapper multi_robot_mapper;
  while(ros::ok()){

      if(ros_handle.ros_data_.isNewImage()) single_robot_mapper.updateMap(ros_handle);

      //if any of the robots are nearby, update map multi_robot_mapper


      ros_handle.ros_data_.resetBools();
      //single_robot_mapper.publishImagewithIDs();
      ros::spinOnce();

  }

  return 0;
}
