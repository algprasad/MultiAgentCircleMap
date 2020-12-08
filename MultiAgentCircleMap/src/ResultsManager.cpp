//
// Created by alg on 05/12/20.
//

#include "MultiAgentCircleMap/ResultsManager.h"
#include <ros/ros.h>

void ResultsManager::writeLandmark2File(int num_landmarks){
    num_landmarks_fout_ << num_landmarks<<","<<ros::Time::now().sec + (ros::Time::now().nsec)*1e-9<<"\n";

}
