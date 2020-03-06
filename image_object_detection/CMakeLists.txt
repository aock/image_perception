cmake_minimum_required(VERSION 3.4)
project(image_object_detection)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse4.1 -O3")

find_package(OpenCV 3.4.3 REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)

set(ROS_DEPS
    roscpp
    rospy
    sensor_msgs
    image_transport
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
        Boost
        OpenCV
    LIBRARIES
        image_object_detection
)

# LIBRARY
add_library(image_object_detection
    src/ObjectDetector.cpp
)

target_include_directories(image_object_detection PUBLIC
    include
    ${image_perception_msgs_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_dependencies(image_object_detection
    ${image_perception_msgs_EXPORTED_TARGETS}
)

target_link_libraries(image_object_detection
    ${OpenCV_LIBS}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
)

# EXECUTABLE
add_executable(image_object_detection_node
    src/image_object_detection_node.cpp
)

target_include_directories(image_object_detection_node PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)

add_dependencies(image_object_detection_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(image_object_detection_node
    ${catkin_LIBRARIES}
    image_object_detection
)

add_executable(image_object_visualization_node
    src/image_object_visualization_node.cpp
)

target_include_directories(image_object_visualization_node PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_dependencies(image_object_visualization_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(image_object_visualization_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)

# INSTALLING
install(TARGETS image_object_detection
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
