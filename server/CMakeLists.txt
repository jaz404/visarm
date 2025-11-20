cmake_minimum_required(VERSION 3.10)
project(visarm_calibration)

find_package(VISP REQUIRED)
if(VISP_FOUND)
  include(${VISP_USE_FILE})
endif(VISP_FOUND)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${VISP_INCLUDE_DIRS})

add_executable(calibrate calibration/calibrate.cpp kinematics/visarm.cpp)
add_executable(main kinematics/mainVisarm.cpp kinematics/visarm.cpp)
target_link_libraries(calibrate ${VISP_LIBRARIES})
target_link_libraries(main ${VISP_LIBRARIES})