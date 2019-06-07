cmake_minimum_required(VERSION 2.8.3)
project(teleoperation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  drive_msgs
  roscpp
  std_msgs
  roslib
  dynamic_reconfigure
)
find_package(SDL2 REQUIRED)
string(STRIP ${SDL2_LIBRARIES} SDL2_LIBRARIES)
set(LIBS ${SDL2_LIBRARIES})

## Errors and Warnings
set ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wchar-subscripts -Wchkp -Wdouble-promotion -Wformat -Wnonnull -Wmain -Wswitch-bool -Winvalid-memory-model -Wunknown-pragmas -Warray-bounds -Wfloat-equal -Wlogical-op -Wpacked ")
# -Wpedantic cant be used because of ROS

#########################
## Dynamic Reconfigure ##
#########################

generate_dynamic_reconfigure_options(
  cfg/joystick_controller.cfg
  cfg/keyboard_controller.cfg
)

###################################
## catkin specific configuration ##
###################################

## Declare things to be passed to dependent projects
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS drive_msgs roscpp std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${SDL2_INCLUDE_DIR}
)

# Declare joystick controller node executable
add_executable(joystick_controller src/joystick_controller.cpp)
target_link_libraries(joystick_controller ${LIBS} ${catkin_LIBRARIES})
add_dependencies(joystick_controller ${${PROJECT_NAME}_EXPORTED_TARGETS}_gencfg)
add_dependencies(joystick_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Declare remote keyboard node executable
add_executable(keyboard_controller src/keyboard_controller.cpp)
target_link_libraries(keyboard_controller ${LIBS} ${catkin_LIBRARIES})
add_dependencies(keyboard_controller ${${PROJECT_NAME}_EXPORTED_TARGETS}_gencfg)
add_dependencies(keyboard_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
