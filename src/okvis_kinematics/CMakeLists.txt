cmake_minimum_required(VERSION 2.8.11)

# require Eigen3
#find_package( Eigen REQUIRED )
#include_directories(${EIGEN_INCLUDE_DIR}) 

# build the library
add_library(okvis_kinematics STATIC 
            src/dependency-tracker.cc
)

