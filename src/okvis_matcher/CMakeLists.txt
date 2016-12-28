cmake_minimum_required(VERSION 2.8.3)

# build the library 
add_library(okvis_matcher
  src/DenseMatcher.cpp
  src/MatchingAlgorithm.cpp
  src/ThreadPool.cpp
  include/okvis/DenseMatcher.hpp
  include/okvis/MatchingAlgorithm.hpp
  include/okvis/ThreadPool.hpp
)

