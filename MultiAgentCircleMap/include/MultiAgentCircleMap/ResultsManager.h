//
// Created by alg on 05/12/20.
//

#ifndef MULTIAGENTCIRCLEMAP_RESULTSMANAGER_H
#define MULTIAGENTCIRCLEMAP_RESULTSMANAGER_H

#include <iostream>
#include <fstream>
#include <ctime>
#include "CircleVec.h"

class ResultsManager {
public:
    std::string num_landmarks_filename_;
    std::ofstream num_landmarks_fout_;
    int robot_index_;

    ResultsManager(int robot_index, std::string dir_path){
        robot_index_ = robot_index;
        time_t now = time(0); //current time and date
        char* dt = (ctime(&now));
        std::string timestamp(dt);
        num_landmarks_filename_ = dir_path + "landmarks" + std::to_string(robot_index_) + timestamp + ".txt";
        num_landmarks_fout_.open(num_landmarks_filename_, std::ios::out );

    }

   /* ~ResultsManager(){
        //close all files
        num_landmarks_fout_.close();
    }*/

    void writeLandmark2File(int num_landmarks);

};


#endif //MULTIAGENTCIRCLEMAP_RESULTSMANAGER_H
