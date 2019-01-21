cmake_minimum_required(VERSION 2.8.3)
project(car_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++1y)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  drive_msgs  
  roscpp
  rospy
  std_msgs
  vesc_sim
)

## Errors and Warnings
set ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wchar-subscripts -Wchkp -Wdouble-promotion -Wformat -Wnonnull -Wmain -Wswitch-bool -Winvalid-memory-model -Wunknown-pragmas -Warray-bounds -Wfloat-equal -Wlogical-op -Wpacked ")
# -Wpedantic cant be used because of ROS

###################################
## catkin specific configuration ##
###################################

## Declare things to be passed to dependent projects
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs drive_msgs vesc_sim
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare car controller node executable
add_executable(car_controller src/car_controller.cpp)
target_link_libraries(car_controller ${catkin_LIBRARIES})
add_dependencies(car_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Declare dms controller node executable
add_executable(dms_controller src/dms_controller.cpp)
target_link_libraries(dms_controller ${catkin_LIBRARIES})
add_dependencies(dms_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Declare Drive Parameters Multiplexer node executable
add_executable(drive_parameters_multiplexer src/drive_parameters_multiplexer.cpp src/drive_parameters_source.cpp)
target_link_libraries(drive_parameters_multiplexer ${catkin_LIBRARIES})
add_dependencies(drive_parameters_multiplexer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test test/test_car_control.cpp)
