cmake_minimum_required(VERSION 2.8.11)

cmake_policy(SET CMP0054 OLD) # cmake 3.1以后版本新特性

# build the library
add_library(okvis_cv STATIC 
  src/CameraBase.cpp
  src/NCameraSystem.cpp
)


