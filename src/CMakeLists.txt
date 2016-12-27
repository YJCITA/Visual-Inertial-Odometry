cmake_minimum_required(VERSION 2.8.11)

add_executable(okvis_app okvis_apps/src/okvis_app_synchronous.cpp)
target_link_libraries(okvis_app 
                    -Wl,--start-group
                    agast
                    brisk
                    ceres
                    opengv
                    okvis_util
                    okvis_kinematics
                    okvis_time
                    okvis_cv 
                    okvis_common
                    okvis_ceres
                    okvis_timing
                    okvis_matcher
                    okvis_frontend 
                    okvis_multisensor_processing
                    pthread 
                    ${OpenCV_LIBRARIES}
                    ${Eigen_LIBS} 
                    ${CERES_LIBRARIES}
                    ${GLOG_LIBRARY}
                    ${Boost_LIBRARIES}
                    -Wl,--end-group
)

 


