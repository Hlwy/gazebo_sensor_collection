cmake_minimum_required(VERSION 2.8.3)
project(gazebo_sensor_collection)

## Add support for C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

set( ROS_CXX_DEPENDENCIES
     roscpp
     sensor_msgs
     geometry_msgs
     visualization_msgs
     std_msgs
     std_srvs
     dynamic_reconfigure
     message_generation
     message_runtime
     nav_msgs
     gazebo_dev
     gazebo_plugins
     gazebo_ros
     gazebo_msgs
     tf
     urdf
     xacro
     cv_bridge
     image_transport
     message_filters
     camera_info_manager
     genmsg
     hector_gazebo_plugins
)

# Excerpt from gazebo_ros_plugins package repo
include (FindPkgConfig)
# include (SwarmbotsConfig.user.cmake)
if (PKG_CONFIG_FOUND)
     pkg_check_modules(XML libxml-2.0)
     pkg_check_modules(OGRE OGRE)
     pkg_check_modules(OGRE-Terrain OGRE-Terrain)
     pkg_check_modules(OGRE-Paging OGRE-Paging)
else()
     message(FATAL_ERROR "pkg-config is required; please install it")
endif()

find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES})
find_package(Boost REQUIRED COMPONENTS thread)
find_package(gazebo REQUIRED)
find_package(Threads REQUIRED)

execute_process(COMMAND
     pkg-config --variable=plugindir OGRE
     OUTPUT_VARIABLE OGRE_PLUGIN_PATH
     OUTPUT_STRIP_TRAILING_WHITESPACE
)

include_directories(
     include
     ${ADDITIONAL_INCLUDE_DIRS}
     ${catkin_INCLUDE_DIRS}
     ${Boost_INCLUDE_DIRS}
     ${GAZEBO_INCLUDE_DIRS}
     ${OGRE_INCLUDE_DIRS}
     ${OGRE-Terrain_INCLUDE_DIRS}
     ${OGRE-Paging_INCLUDE_DIRS}
)
link_directories(
     ${GAZEBO_LIBRARY_DIRS}
     ${OGRE_LIBRARY_DIRS}
     ${OGRE-Terrain_LIBRARY_DIRS}
     ${OGRE-Paging_LIBRARY_DIRS}
)


#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
     FILES
     DataXYZ.msg
     EulerData.msg
     QuaternionData.msg
     SensorData3D.msg
     OrientationSensorData.msg
     ImuData.msg
     GpsData.msg
     EncoderData.msg
     BatteryData.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
     DEPENDENCIES
     geometry_msgs
     visualization_msgs
     gazebo_sensor_collection
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
     INCLUDE_DIRS include
     LIBRARIES gazebo_sensor_collection
     CATKIN_DEPENDS ${ROS_CXX_DEPENDENCIES}
     DEPENDS
     Boost
     roscpp
     gazebo_ros
)
add_dependencies(${catkin_EXPORTED_TARGETS})

###################
## Build Plugins ##
###################

# Hector-gazebo repo based plugins
add_library(gazebo_ros_imu_mod plugins/motion/gazebo_ros_imu_mod.cpp)
target_link_libraries(gazebo_ros_imu_mod ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(gazebo_ros_imu_mod ${PROJECT_NAME}_generate_messages_cpp)

add_library(gazebo_ros_gps_with_dropout plugins/gps/gazebo_ros_gps_with_dropout.cpp)
target_link_libraries(gazebo_ros_gps_with_dropout ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(gazebo_ros_gps_with_dropout ${PROJECT_NAME}_generate_messages_cpp)

add_library(gazebo_ros_skid_steer_w_encoder plugins/motion/skid_steering_with_encoder.cpp)
target_link_libraries(gazebo_ros_skid_steer_w_encoder ${catkin_LIBRARIES})
add_dependencies(gazebo_ros_skid_steer_w_encoder ${PROJECT_NAME}_generate_messages_cpp)

add_library(gazebo_ros_encoder plugins/motion/gazebo_encoder.cpp)
target_link_libraries(gazebo_ros_encoder ${catkin_LIBRARIES})
add_dependencies(gazebo_ros_encoder ${PROJECT_NAME}_generate_messages_cpp)

add_library(gazebo_ros_battery_simple plugins/misc/gazebo_ros_battery_simple.cpp)
target_link_libraries(gazebo_ros_battery_simple ${catkin_LIBRARIES})
add_dependencies(gazebo_ros_battery_simple ${PROJECT_NAME}_generate_messages_cpp)

add_library(gazebo_ros_realsense plugins/vision/gazebo_ros_realsense.cpp)
target_link_libraries(gazebo_ros_realsense ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(gazebo_ros_realsense ${catkin_EXPORTED_TARGETS})
