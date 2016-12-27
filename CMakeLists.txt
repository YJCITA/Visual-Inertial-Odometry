cmake_minimum_required(VERSION 2.8.11)
project(okvis)

SET(PROJECT_ROOT_PATH "${CMAKE_SOURCE_DIR}")
MESSAGE(STATUS ${PROJECT_ROOT_PATH})

SET(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin")
SET(LIBRARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/lib/")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -Wall -std=c++11 -fPIC")


# require OpenCV
find_package( OpenCV COMPONENTS core highgui imgproc features2d REQUIRED )
include_directories(BEFORE ${OpenCV_INCLUDE_DIRS}) 

# require okvis
SET(okvis_DIR "/home/yj/bak/packet/okvis_install/lib/CMake/okvis/")
find_package( okvis 1.1 REQUIRED)
include_directories(${OKVIS_INCLUDE_DIRS})
#MESSAGE(STATUS "22222---OKVIS_INCLUDE_DIRS = ${OKVIS_INCLUDE_DIRS}")

# require brisk
SET(brisk_DIR "/home/yj/bak/packet/okvis_install/lib/CMake/brisk/")
find_package( brisk 2 REQUIRED)
include_directories(${BRISK_INCLUDE_DIRS})
#MESSAGE(STATUS "22222---BRISK_INCLUDE_DIRS = ${BRISK_INCLUDE_DIRS}")

# require ceres
list(APPEND CMAKE_PREFIX_PATH ${OKVIS_INSTALLATION})
find_package( Ceres REQUIRED )
include_directories(${CERES_INCLUDE_DIRS}) 
#MESSAGE(STATUS "22222---CERES_INCLUDE_DIRS = ${CERES_INCLUDE_DIRS}")

# require OpenGV
SET(opengv_DIR "/home/yj/bak/packet/okvis_install/lib/CMake/opengv/")
find_package(opengv REQUIRED)
#MESSAGE(STATUS "22222---opengv_INCLUDE_DIRS = ${opengv_INCLUDE_DIRS}")

# VISensor, if available
#list(APPEND CMAKE_MODULE_PATH ${OKVIS_INSTALLATION}/lib/CMake)
#find_package(VISensor)
#if(VISENSORDRIVER_FOUND)
#  message(STATUS "Found libvisensor.")
#else()
#  message(STATUS "libvisensor not found")
#endif()

FIND_PACKAGE(Boost COMPONENTS filesystem system REQUIRED)
include_directories(${Boost_INCLUDE_DIRS}) 

ADD_SUBDIRECTORY(./src)

