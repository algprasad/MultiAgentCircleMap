//
// Created by alg on 21/11/20.
//

#include "MultiAgentCircleMap/MultiRobotMapper.h"

void MultiRobotMapper::updateMap(MultiAgentCircleMap::RosHandle ros_handle, SingleRobotMapper &single_robot_mapper) {
    //ros_handle has the data for the new circles from other robots and single_robot_mapper has the global_circles and the function to update the map

    /*if(ros_handle.ros_data_.new_global_circle0_ && ros_handle.ros_data_.global_circle0_array_.index!= this->robot_index_){
        CircleVec other_robot_global_circles(ros_handle.ros_data_.global_circle0_array_);
        single_robot_mapper.updateMap(other_robot_global_circles);

    }

    if( ros_handle.ros_data_.new_global_circle1_ && ros_handle.ros_data_.global_circle1_array_.index!= this->robot_index_){
        CircleVec other_robot_global_circles(ros_handle.ros_data_.global_circle1_array_);
        single_robot_mapper.updateMap(other_robot_global_circles);

    }

    if(ros_handle.ros_data_.new_global_circle2_ && ros_handle.ros_data_.global_circle2_array_.index!= this->robot_index_){
        CircleVec other_robot_global_circles(ros_handle.ros_data_.global_circle2_array_);
        single_robot_mapper.updateMap(other_robot_global_circles);

    }*/
    if(ros_handle.ros_data_.new_global_circle0_){
        CircleVec other_robot_global_circles(ros_handle.ros_data_.global_circle0_array_);
        single_robot_mapper.updateMap(other_robot_global_circles);
    }
    if(ros_handle.ros_data_.new_global_circle1_){
        CircleVec other_robot_global_circles(ros_handle.ros_data_.global_circle1_array_);
        single_robot_mapper.updateMap(other_robot_global_circles);
    }
    if(ros_handle.ros_data_.new_global_circle2_){
        CircleVec other_robot_global_circles(ros_handle.ros_data_.global_circle2_array_);
        single_robot_mapper.updateMap(other_robot_global_circles);
    }
    if(ros_handle.ros_data_.new_global_circle3_){
        CircleVec other_robot_global_circles(ros_handle.ros_data_.global_circle3_array_);
        single_robot_mapper.updateMap(other_robot_global_circles);
    }
    if(ros_handle.ros_data_.new_global_circle4_){
        CircleVec other_robot_global_circles(ros_handle.ros_data_.global_circle4_array_);
        single_robot_mapper.updateMap(other_robot_global_circles);
    }



}
