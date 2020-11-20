#include <ros/ros.h>
#include "MultiAgentCircleMap/RosHandle.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "MultiAgentCircleMap");
  ros::NodeHandle node_handle("~");


  MultiAgentCircleMap::RosHandle ros_handle(node_handle); 

  while(ros::ok()){
      
      ros::spinOnce();
  }

  return 0;
}
