cmake_minimum_required(VERSION 3.0.2)
project(mimic_controllers)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  name_sorting
  roscpp
  sensor_msgs
  subscription_notifier
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS controller_interface hardware_interface name_sorting roscpp sensor_msgs subscription_notifier
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME}
 src/${PROJECT_NAME}/mimic_controller.cpp
 src/${PROJECT_NAME}/mimic_effort_controller.cpp
 src/${PROJECT_NAME}/two_fingers_controller.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
