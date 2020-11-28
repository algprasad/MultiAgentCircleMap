cmake_minimum_required(VERSION 3.5)
project(MultiAgentCircleMap)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.

#add_definitions(-Wall -Werror)
# Using Eigen from the GTSAM library to avoid headaches


## Find catkin macros and libraries
##
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    sensor_msgs
    roslint image_transport cv_bridge sensor_msgs message_generation
)
#find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(yaml-cpp REQUIRED)


add_message_files(
		FILES
		CircleMsg.msg
		CircleArray.msg
)

generate_messages(
		DEPENDENCIES
		std_msgs  # Or other packages containing msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
    LIBRARIES
    #${PROJECT_NAME}_core
    CATKIN_DEPENDS
    roscpp
    sensor_msgs
  DEPENDS
  ## find_package(Eigen3) provides a non standard EIGEN3_INCLUDE_DIR instead of Eigen3_INCLUDE_DIRS.
  ## Therefore, the DEPEND does not work as expected and we need to add the directory to the INCLUDE_DIRS
  # Eigen3

  ## Boost is not part of the DEPENDS since it is only used in source files,
  ## Dependees do not depend on Boost when they depend on this package.
)




###########
## Build ##
###########
# Set all auxillary header files here
set(AUX_SRC_FILES include/MultiAgentCircleMap/RosData.h
               )


## Specify additional locations of header files
## Your package locations should be listed before other locations
message("CMake include dires")
message("${CMAKE_CURRENT_BINARY_DIR}")
include_directories(
  include
		#dont need this for catkin build only for CLion . #TODO(OFN): Find a way around this
  /home/alg/RoverLocalization/rover_localization_ws/devel/.private/MultiAgentCircleMap/include
  ${catkin_INCLUDE_DIRS}
  # Set manually because Eigen sets a non standard INCLUDE DIR
        #${GTSAM_EIGEN_INCLUDE_DIRS}
  # Set because Boost is an internal dependency, not transitive.
  ${Boost_INCLUDE_DIRS} #${GTSAM_INCLUDE_DIR}
)
message("CMake include dires")
message("${INCLUDE_DIRECTORIES}")
get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
	message(STATUS "dir='${dir}'")
endforeach()


#link_directories("${CMAKE_CURRENT_SOURCE_DIR}/../ThirdParty/aruco3.1.6/build/src")
## Declare a cpp library #TODO: Maybe remove the AUX_SRC_FILES from here
#add_library(${PROJECT_NAME}_core
#    ${AUX_SRC_FILES}
#)
#target_compile_features(${PROJECT_NAME}_core INTERFACE cxx_std_14)

## Declare cpp executables
## Executable 1: Iris0
set(IRIS0 ${PROJECT_NAME}_iris0)
set(IRIS1 ${PROJECT_NAME}_iris1)
set(IRIS2 ${PROJECT_NAME}_iris2) ## TODO: do the following three steps for all the drones

add_executable(${IRIS0}
  src/${PROJECT_NAME}_node_iris0.cpp
        src/RosHandle.cpp
        src/Image.cpp include/MultiAgentCircleMap/Image.h src/Circle.cpp include/MultiAgentCircleMap/Circle.h src/CircleVec.cpp include/MultiAgentCircleMap/CircleVec.h src/Graph.cpp include/MultiAgentCircleMap/Graph.h src/SingleRobotMapper.cpp include/MultiAgentCircleMap/SingleRobotMapper.h src/MultiRobotMapper.cpp include/MultiAgentCircleMap/MultiRobotMapper.h src/Hungarian.cpp include/MultiAgentCircleMap/Hungarian.h src/Utils.h src/Visualization.cpp include/MultiAgentCircleMap/Visualization.h)

add_dependencies(${IRIS0}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${IRIS0}
#  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES} ${OpenCV_LIBS}
        yaml-cpp 
	#aruco gtsam
)

add_executable(${IRIS1}
		src/${PROJECT_NAME}_node_iris1.cpp
		src/RosHandle.cpp
		src/Image.cpp include/MultiAgentCircleMap/Image.h src/Circle.cpp include/MultiAgentCircleMap/Circle.h src/CircleVec.cpp include/MultiAgentCircleMap/CircleVec.h src/Graph.cpp include/MultiAgentCircleMap/Graph.h src/SingleRobotMapper.cpp include/MultiAgentCircleMap/SingleRobotMapper.h src/MultiRobotMapper.cpp include/MultiAgentCircleMap/MultiRobotMapper.h src/Hungarian.cpp include/MultiAgentCircleMap/Hungarian.h src/Visualization.cpp include/MultiAgentCircleMap/Visualization.h)

add_dependencies(${IRIS1}
		${catkin_EXPORTED_TARGETS}
		)

target_link_libraries(${IRIS1}
		#  ${PROJECT_NAME}_core
		${catkin_LIBRARIES} ${OpenCV_LIBS}
		yaml-cpp
		#aruco gtsam
		)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
#install(
#  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_core
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

# Mark cpp header files for installation
install(
  DIRECTORY include/${IRIS0}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

# Mark other files for installation
install(
  DIRECTORY doc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if(${CATKIN_ENABLE_TESTING})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
## Add gtest based cpp test target and link libraries
catkin_add_gtest(${IRIS0}-test
        test/test_MultiAgentCircleMap.cpp
	)

#target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_core)
endif()

##########################
## Static code analysis ##
##########################

roslint_cpp()
