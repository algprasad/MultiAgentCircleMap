//
// Created by alg on 21/11/20.
//

#include "MultiAgentCircleMap/SingleRobotMapper.h"
#include "MultiAgentCircleMap/Hungarian.h"
#define DEBUG 0

/** Cost matrix based on Pixel distance */
std::vector<std::vector<double> > SingleRobotMapper::getPixelCostMatrixHungarianAlgo(double threshold_pixel_distance){
    double THRESHOLD_PIXEL_DISTANCE = threshold_pixel_distance;
    std::vector<std::vector<double> > cost_matrix;
    for(int i =0; i< prev_image_.getSize(); i++){
        std::vector<double> temp_cost_row;
        for(int j =0; j < current_image_.getSize(); j++){

            Eigen::Vector2d previous_point = prev_image_.circle_vec_.circle_vec_[i].pixel_coords_.top();
            Eigen::Vector2d current_point = current_image_.circle_vec_.circle_vec_[j].pixel_coords_.top();

            //get distance between the two points
            double distance = sqrt(pow((previous_point[0] - current_point[0]), 2) + pow((previous_point[1] - current_point[1]), 2));
            if(distance > THRESHOLD_PIXEL_DISTANCE) distance = 2000;
            temp_cost_row.push_back(distance);

        }
        cost_matrix.push_back(temp_cost_row);

    }
    return cost_matrix;
}



void SingleRobotMapper::updateMap(MultiAgentCircleMap::RosHandle ros_handle) {
    current_image_ = ros_handle.ros_data_.image_;
    if(this->first_image_){
        global_circles_vec_.circle_vec_ = current_image_.circle_vec_.circle_vec_;
        first_image_ = false;
    }
    else{

        //get cost matrix for the Hungarian Algo
        std::vector<std::vector<double> > cost_matrix_hung = getPixelCostMatrixHungarianAlgo(ros_handle.threshold_pixel_distance_);

        //main Hungarian Algo step
        std::vector<int > assignment = getAssignment(cost_matrix_hung);


        //after getting the assignment,  assign the IDs of the circles equal to their counter parts in the previous_circles
        assignCorrepondingPrevId(assignment, cost_matrix_hung);

        //assign new ID based for all the new landmarks.. //IMP: Also takes care of the adding new circles to the global_circles
        assignNewID();

    }

    this->publishImagewithIDs(ros_handle);
    if(DEBUG && global_circles_vec_.circle_vec_.size() > 5) {
        std::cout<<global_circles_vec_.circle_vec_.size()<<std::endl;
        for(int i=0; i< 5; i++){
            std::cout<<"Global Position: "<<global_circles_vec_.circle_vec_[i].global_position_<<std::endl;
        }
    }

    prev_image_ = current_image_;
}

std::vector<int> SingleRobotMapper::getAssignment(std::vector<std::vector<double> > &cost_matrix) {
    HungarianAlgorithm hungarian_algorithm;
    std::vector<int > assignment;
    double cost  = hungarian_algorithm.Solve(cost_matrix, assignment);
    return assignment;

}

void SingleRobotMapper::assignCorrepondingPrevId(std::vector<int> &assignment,
                                                 std::vector<std::vector<double> > cost_matrix) {

    for(int i =0; i< assignment.size(); i++){
        int assigned_partner_in_next = assignment[i];
        //assigned partner for previous_circles[i] is current_circle[assigned_partner_in_next]
        if(assigned_partner_in_next!= -1){

            if(cost_matrix[i][assigned_partner_in_next] != 2000 && cost_matrix[i][assigned_partner_in_next] >= 0){
                current_image_.circle_vec_.circle_vec_[assigned_partner_in_next].index_ = prev_image_.circle_vec_.circle_vec_[i].index_;
                current_image_.circle_vec_.circle_vec_[assigned_partner_in_next].setBoolHasIndex(true);
            }
        }
    }

}

void SingleRobotMapper::assignNewID() {
    for(int i =0; i< current_image_.circle_vec_.circle_vec_.size(); i++){
        if(!current_image_.circle_vec_.circle_vec_[i].hasIndex_){ //new circle with previously unknown association
            current_image_.circle_vec_.circle_vec_[i].index_ = global_circles_vec_.circle_vec_.size() + 1 ;
            current_image_.circle_vec_.circle_vec_[i].setBoolHasIndex(true);
            global_circles_vec_.circle_vec_.push_back(current_image_.circle_vec_.circle_vec_[i]); //TAKES CARE OF ADDING NEW CIRCLES TO THE global_circles
        }
    }

}

void SingleRobotMapper::publishImagewithIDs(MultiAgentCircleMap::RosHandle& ros_handle) {
    ros_handle.ros_data_.image_.circle_vec_ = current_image_.circle_vec_;
    ros_handle.ros_data_.image_.writeLandmarkID();
    ros_handle.pubDetectedCircles(current_image_.getImageWithDetectedCircles());
}


