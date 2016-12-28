cmake_minimum_required(VERSION 2.8.11)
cmake_policy(SET CMP0054 OLD) # cmake 3.1以后版本新特性

add_library(okvis_multisensor_processing
  src/ThreadedKFVio.cpp
  src/ImuFrameSynchronizer.cpp
  src/FrameSynchronizer.cpp
  src/VioVisualizer.cpp
  include/okvis/ThreadedKFVio.hpp
  include/okvis/ImuFrameSynchronizer.hpp
  include/okvis/FrameSynchronizer.hpp
  include/okvis/VioVisualizer.hpp
  include/okvis/threadsafe/ThreadsafeQueue.hpp

)


