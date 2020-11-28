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

void SingleRobotMapper::HungarianAssignment(MultiAgentCircleMap::RosHandle ros_handle){

    //get cost matrix for the Hungarian Algo
    std::vector<std::vector<double> > cost_matrix_hung = getPixelCostMatrixHungarianAlgo(ros_handle.threshold_pixel_distance_);

    //main Hungarian Algo step
    std::vector<int > assignment = getAssignment(cost_matrix_hung);


    //after getting the assignment,  assign the IDs of the circles equal to their counter parts in the previous_circles
    assignCorrepondingPrevId(assignment, cost_matrix_hung);

    //assign new ID based for all the new landmarks.. //IMP: Also takes care of the adding new circles to the global_circles
    assignNewID();

}

void SingleRobotMapper::assignGlobalID2CurrentVec(std::vector<int> assignment_vec){
    for(int i =0; i<current_image_.circle_vec_.circle_vec_.size(); i++){
        if(assignment_vec[i] != -1){ //that means there is an assignment
            //set the landmark ID equal to the ID of its global counter part //mostly for visualization purposes
            current_image_.circle_vec_.circle_vec_[i].setIndex(global_circles_vec_.circle_vec_[assignment_vec[i]].index_);
            current_image_.circle_vec_.circle_vec_[i].setBoolHasIndex(true);
        }
        else{ //i.e. if assignment_vec[i] == -1
            //assign new ID
            current_image_.circle_vec_.circle_vec_[i].index_ = global_circles_vec_.circle_vec_.size() + 1 ;
            current_image_.circle_vec_.circle_vec_[i].setBoolHasIndex(true);
            global_circles_vec_.circle_vec_.push_back(current_image_.circle_vec_.circle_vec_[i]);

        }
    }

}


void SingleRobotMapper::NearestNeighbourAssignment() {
    //std::cout<<"Inside Nearest Neighbour";
    double POSITION_DIST_THRESHOLD = 0.5;
    std::vector<int> assignment_vec(current_image_.circle_vec_.circle_vec_.size());
    for(int i =0; i< current_image_.circle_vec_.circle_vec_.size(); i++){
        double min_distance = INT64_MAX;
        double assignment = -1;
        for(int j =0; j< global_circles_vec_.circle_vec_.size(); j++){
            Eigen::Vector3d temp_diff_vec  = current_image_.circle_vec_.circle_vec_[i].global_position_  - global_circles_vec_.circle_vec_[j].global_position_;
            double distance = temp_diff_vec.norm();
            if(distance < min_distance && distance < POSITION_DIST_THRESHOLD) {
                min_distance = distance;
                assignment = j;
            }
        }
        assignment_vec[i] = assignment;
    }

    //TODO function to remove one circle assigned to multiple global circles

    //function to take assign IDs (global IDs for recurring landmarks and new IDs to new landmarks)
    assignGlobalID2CurrentVec(assignment_vec);

}


void SingleRobotMapper::updateMap(MultiAgentCircleMap::RosHandle ros_handle) {
    current_image_ = ros_handle.ros_data_.image_;
    if(this->first_image_){
        global_circles_vec_.circle_vec_ = current_image_.circle_vec_.circle_vec_;
        first_image_ = false;
    }
    else{
        if(current_image_.used_pixels_) HungarianAssignment(ros_handle);  //uses pixel coordinates
        else NearestNeighbourAssignment();  //uses global position coordiantes of the circle centres
    }

    this->publishImagewithIDs(ros_handle);
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
    ros_handle.pubDetectedCirclesImage(current_image_.getImageWithDetectedCircles());
}




