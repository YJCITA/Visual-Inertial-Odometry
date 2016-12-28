cmake_minimum_required(VERSION 2.8.11)
project(okvis)

######################### configuration ###########################
#MESSAGE(STATUS " copmile debug")
#SET(CMAKE_BUILD_TYPE "Debug")
#SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")

SET(PROJECT_ROOT_PATH "${CMAKE_SOURCE_DIR}")
MESSAGE(STATUS ${PROJECT_ROOT_PATH})

SET(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin")
SET(LIBRARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/lib")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -Wall -std=c++11 -fPIC")

# set okvis lib add
SET(OKVIS_LIB_ADDR "${CMAKE_SOURCE_DIR}/lib/okvis_lib")

# require OpenCV
find_package( OpenCV COMPONENTS core highgui imgproc features2d REQUIRED )
include_directories(BEFORE ${OpenCV_INCLUDE_DIRS})

# require okvis
#SET(okvis_DIR "${OKVIS_LIB_ADDR}/lib/CMake/okvis/")
#find_package( okvis 1.1 REQUIRED)
#include_directories(${OKVIS_INCLUDE_DIRS})

# require brisk
SET(brisk_DIR "${OKVIS_LIB_ADDR}/lib/CMake/brisk/")
find_package( brisk 2 REQUIRED)
include_directories(${BRISK_INCLUDE_DIRS})

# require ceres
list(APPEND CMAKE_PREFIX_PATH ${OKVIS_INSTALLATION})
find_package( Ceres REQUIRED )
include_directories(${CERES_INCLUDE_DIRS})

# require OpenGV
SET(opengv_DIR "${OKVIS_LIB_ADDR}/lib/CMake/opengv/")
find_package(opengv REQUIRED)

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

include_directories(./src/agast/include)

include_directories(./src/okvis_util/include)
add_subdirectory(./src/okvis_util)

include_directories(./src/okvis_kinematics/include)
add_subdirectory(./src/okvis_kinematics)

include_directories(./src/okvis_time/include)
add_subdirectory(./src/okvis_time)

include_directories(./src/okvis_cv/include)
add_subdirectory(./src/okvis_cv)

include_directories(./src/okvis_common/include)
add_subdirectory(./src/okvis_common)

include_directories(./src/okvis_matcher/include)
add_subdirectory(./src/okvis_matcher)

include_directories(./src/okvis_multisensor_processing/include)
add_subdirectory(./src/okvis_multisensor_processing)

include_directories(./src/okvis_frontend/include)
add_subdirectory(./src/okvis_frontend)

ADD_SUBDIRECTORY(./src)

