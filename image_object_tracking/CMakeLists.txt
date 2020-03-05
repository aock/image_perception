cmake_minimum_required(VERSION 3.4)
project(image_object_tracking)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse4.1 -O3")

find_package(OpenCV 3.4.3 REQUIRED)

set(ROS_DEPS
    roscpp
    rospy
    image_perception_msgs
)

find_package(catkin REQUIRED
  COMPONENTS
    ${ROS_DEPS}
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
        image_object_tracking
)

# LIBRARY
add_library(image_object_tracking
    src/ObjectTracker.cpp
)

target_include_directories(image_object_tracking PUBLIC
    include
    ${image_perception_msgs_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_dependencies(image_object_tracking
    ${image_perception_msgs_EXPORTED_TARGETS}
)

target_link_libraries(image_object_tracking
    ${OpenCV_LIBS}
)

# EXECUTABLE
add_executable(image_object_tracking_node
    src/image_object_tracking_node.cpp
)

target_include_directories(image_object_tracking_node PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)

add_dependencies(image_object_tracking_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(image_object_tracking_node
    ${catkin_LIBRARIES}
    image_object_tracking
)

# INSTALLING
install(TARGETS image_object_tracking
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
