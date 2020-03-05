cmake_minimum_required(VERSION 3.4)
project(image_perception_msgs)

find_package(OpenCV REQUIRED)

set(ROS_DEPS
    roscpp
    std_msgs
    sensor_msgs
    message_generation
)

find_package(catkin REQUIRED
  COMPONENTS
    ${ROS_DEPS}
)

# MESSAGE GENERATION

add_message_files(
  FILES
  Rect.msg
  Label.msg
  LabelsStamped.msg
  TrackedObject.msg
  TrackedObjectsStamped.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

# EXPORTING
catkin_package(
    INCLUDE_DIRS
        include
    CATKIN_DEPENDS
        ${ROS_DEPS}
    DEPENDS
        OpenCV
    LIBRARIES
        image_perception_msgs
)

# LIBRARY FOR CONVERSION

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_library(image_perception_msgs
    src/Conversions.cpp
)

add_dependencies(image_perception_msgs
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

# INSTALLING
install(TARGETS image_perception_msgs
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
