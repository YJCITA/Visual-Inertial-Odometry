cmake_minimum_required(VERSION 2.8.11)

# build the library
add_library(okvis_common STATIC 
  src/VioInterface.cpp
  src/VioParametersReader.cpp
  include/okvis/FrameTypedefs.hpp
  include/okvis/Measurements.hpp
  include/okvis/Parameters.hpp
  include/okvis/Variables.hpp
  include/okvis/VioBackendInterface.hpp
  include/okvis/VioFrontendInterface.hpp
  include/okvis/VioInterface.hpp
  include/okvis/VioParametersReader.hpp
)

