cmake_minimum_required(VERSION 3.5)
project(gazebo_sensor_collection)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-dev -Wextra -Wpedantic -Wno-inconsistent-missing-override -Wno-unused-but-set-variable -Wno-sign-compare -Wno-return-type")
endif()
# set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
# find_package(octomap REQUIRED)
# find_package(gazebo REQUIRED)
find_package(gazebo_dev REQUIRED)
# find_package(gazebo_ros REQUIRED)
find_package(gazebo_plugins REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)

# ## Find gazebo
# if(POLICY CMP0054)
#     cmake_policy(SET CMP0054 NEW)
# endif()
find_package(gazebo REQUIRED)
message(STATUS "Gazebo version: ${GAZEBO_VERSION}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
message(STATUS "Gazebo flags: ${GAZEBO_CXX_FLAGS}")
message(STATUS "Gazebo deps: ${GAZEBO_LIBRARY_DIRS}")
message(STATUS "Gazebo_dev deps: ${gazebo_dev_LIBRARIES}")
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# message(STATUS "tf2 deps: ---  ${tf2_LIBRARIES}")
# message(STATUS "tf2_ros deps: ---  ${tf2_ros_LIBRARIES}")
# message(STATUS "cv_bridge deps: ---  ${cv_bridge_LIBRARIES}")
# message(STATUS "octomap deps: ---  ${octomap_LIBRARIES}")
# message(STATUS "gazebo_plugins deps: ---  ${gazebo_plugins_LIBRARIES}")
# message(STATUS "image_transport deps: ---  ${image_transport_LIBRARIES}")
# message(STATUS "camera_info_manager deps: ---  ${camera_info_manager_LIBRARIES}")
# message(STATUS "gazebo_msgs deps: ---  ${gazebo_msgs_LIBRARIES}")
# message(STATUS "trajectory_msgs deps: ---  ${trajectory_msgs_LIBRARIES}")
# message(STATUS "tf2_geometry_msgs deps: ---  ${tf2_geometry_msgs_LIBRARIES}")
# message(STATUS "geographic_msgs deps: ---  ${geographic_msgs_LIBRARIES}")

# # Excerpt from gazebo_ros_plugins package repo
# include (FindPkgConfig)
# # include (SwarmbotsConfig.user.cmake)
# if (PKG_CONFIG_FOUND)
#      pkg_check_modules(XML libxml-2.0)
#      pkg_check_modules(OGRE OGRE)
#      pkg_check_modules(OGRE-Terrain OGRE-Terrain)
#      pkg_check_modules(OGRE-Paging OGRE-Paging)
# else()
#      message(FATAL_ERROR "pkg-config is required; please install it")
# endif()
#
# find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES} )
# find_package(Boost REQUIRED COMPONENTS thread)
# find_package(gazebo REQUIRED)
# find_package(Threads REQUIRED)
#
# execute_process(COMMAND
#      pkg-config --variable=plugindir OGRE
#      OUTPUT_VARIABLE OGRE_PLUGIN_PATH
#      OUTPUT_STRIP_TRAILING_WHITESPACE
# )
#
# include_directories(
#      include
#      ${ADDITIONAL_INCLUDE_DIRS}
#      ${catkin_INCLUDE_DIRS}
#      ${Boost_INCLUDE_DIRS}
#      ${GAZEBO_INCLUDE_DIRS}
#      ${OGRE_INCLUDE_DIRS}
#      ${OGRE-Terrain_INCLUDE_DIRS}
#      ${OGRE-Paging_INCLUDE_DIRS}
# )
# link_directories(
#      ${GAZEBO_LIBRARY_DIRS}
#      ${OGRE_LIBRARY_DIRS}
#      ${OGRE-Terrain_LIBRARY_DIRS}
#      ${OGRE-Paging_LIBRARY_DIRS}
# )


## Set includes
set(INCLUDE_DIRS include
    ${sensor_msgs_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}/srv
)
include_directories(${INCLUDE_DIRS})

set(LIBS
     visualization_msgs
    geographic_msgs
    geometry_msgs
    sensor_msgs
    gazebo_ros
    std_msgs
    std_srvs
    tf2
    rclcpp
)

#######################################
## Declare ROS messages and services ##
#######################################

set(msg_files
     "msg/DataXYZ.msg"
     "msg/EulerData.msg"
     "msg/QuaternionData.msg"
     "msg/SensorData3D.msg"
     "msg/OrientationSensorData.msg"
     "msg/ImuData.msg"
     "msg/GpsData.msg"
     "msg/EncoderData.msg"
     "msg/BatteryData.msg"
)
# generate_dynamic_reconfigure_options( cfg/SimulatedCameraD4XX.cfg )

## Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    srv/SetBias.srv
    srv/SetReferenceGeoPose.srv
    DEPENDENCIES
        builtin_interfaces
        std_msgs
        nav_msgs
        sensor_msgs
        # gazebo_msgs
        geometry_msgs
        geographic_msgs
        trajectory_msgs
        tf2_geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)


###################################
## catkin specific configuration ##
###################################

# catkin_package(
#      INCLUDE_DIRS include
#      LIBRARIES gazebo_sensor_collection
#      CATKIN_DEPENDS ${ROS_CXX_DEPENDENCIES}
#      DEPENDS
#      Boost
#      roscpp
#      gazebo_ros
# )
# add_dependencies(${catkin_EXPORTED_TARGETS})

###################
## Build Plugins ##
###################
set(BUILT_GAZEBO_ROS_PLUGINS_LIST "")

# Hector-gazebo repo based plugins
# add_library(gazebo_ros_imu_mod plugins/motion/gazebo_ros_imu_mod.cpp)
# target_link_libraries(gazebo_ros_imu_mod ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
# add_dependencies(gazebo_ros_imu_mod ${PROJECT_NAME}_generate_messages_cpp)

add_library(gazebo_ros_imu_hec_mod SHARED src/motion/gazebo_ros_imu_mod_hec.cpp)
target_link_libraries(gazebo_ros_imu_hec_mod ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES})
list(APPEND BUILT_GAZEBO_ROS_PLUGINS_LIST gazebo_ros_imu_hec_mod)
ament_target_dependencies(gazebo_ros_imu_hec_mod ${LIBS})

add_library(gazebo_ros_gps_hec_mod SHARED src/gps/gazebo_ros_gps_mod_hec.cpp)
target_link_libraries(gazebo_ros_gps_hec_mod ${GAZEBO_LIBRARIES})
list(APPEND BUILT_GAZEBO_ROS_PLUGINS_LIST gazebo_ros_gps_hec_mod)
ament_target_dependencies(gazebo_ros_gps_hec_mod ${LIBS})

add_library(gazebo_ros_gps_with_dropout SHARED src/gps/gazebo_ros_gps_with_dropout.cpp)
target_link_libraries(gazebo_ros_gps_with_dropout ${GAZEBO_LIBRARIES})
list(APPEND BUILT_GAZEBO_ROS_PLUGINS_LIST gazebo_ros_gps_with_dropout)
ament_target_dependencies(gazebo_ros_gps_with_dropout ${LIBS})

# add_library(gazebo_ros_skid_steer_w_encoder SHARED src/motion/skid_steering_with_encoder.cpp)
# # target_include_directories(gazebo_ros_skid_steer_w_encoder PUBLIC include)
# # target_link_libraries(gazebo_ros_skid_steer_w_encoder ${GAZEBO_LIBRARIES})
# ament_target_dependencies(gazebo_ros_skid_steer_w_encoder "rclcpp" "nav_msgs" "geometry_msgs" "tf2" "tf2_geometry_msgs" "tf2_ros" )
# # ament_target_dependencies(gazebo_ros_skid_steer_w_encoder "rclcpp" "gazebo_dev" "gazebo_ros" "nav_msgs" "geometry_msgs" "tf2" "tf2_geometry_msgs" "tf2_ros" )
# ament_export_libraries(gazebo_ros_skid_steer_w_encoder)

# add_library(gazebo_ros_encoder plugins/motion/gazebo_encoder.cpp)
# target_link_libraries(gazebo_ros_encoder ${catkin_LIBRARIES})
# add_dependencies(gazebo_ros_encoder ${PROJECT_NAME}_generate_messages_cpp)
#
# add_library(gazebo_ros_battery_simple plugins/misc/gazebo_ros_battery_simple.cpp)
# target_link_libraries(gazebo_ros_battery_simple ${catkin_LIBRARIES})
# add_dependencies(gazebo_ros_battery_simple ${PROJECT_NAME}_generate_messages_cpp)
#
# add_library(gazebo_ros_realsense plugins/vision/gazebo_ros_realsense.cpp)
# target_link_libraries(gazebo_ros_realsense ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
# add_dependencies(gazebo_ros_realsense ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

# Connect targets with locally defined messages
get_available_rmw_implementations(rmw_implementations2)
foreach(rmw_implementation ${rmw_implementations2})
    find_package("${rmw_implementation}" REQUIRED)
    get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")
    foreach(typesupport_impl ${typesupport_impls})
        rosidl_target_interfaces(gazebo_ros_imu_hec_mod ${PROJECT_NAME} ${typesupport_impl})
        rosidl_target_interfaces(gazebo_ros_gps_hec_mod ${PROJECT_NAME} ${typesupport_impl})
        rosidl_target_interfaces(gazebo_ros_gps_with_dropout ${PROJECT_NAME} ${typesupport_impl})
    endforeach()
endforeach()

#############
## Install ##
#############
# ament_export_include_directories(include)
# ament_export_dependencies(rclcpp)
# # ament_export_dependencies(gazebo)
# # ament_export_dependencies(gazebo_dev)
# ament_export_dependencies(gazebo_msgs)
# # ament_export_dependencies(gazebo_ros)


# install(TARGETS
#      # gazebo_ros_imu_mod
#      # gazebo_ros_gps_with_dropout
#      gazebo_ros_skid_steer_w_encoder
#      # gazebo_ros_encoder
#      # gazebo_ros_battery_simple
#      # gazebo_ros_realsense
#   RUNTIME DESTINATION bin # ${CATKIN_PACKAGE_BIN_DESTINATION}
#   LIBRARY DESTINATION lib # ${CATKIN_PACKAGE_LIB_DESTINATION}
#   ARCHIVE DESTINATION lib # ${CATKIN_PACKAGE_LIB_DESTINATION}
# )

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

install(TARGETS ${BUILT_GAZEBO_ROS_PLUGINS_LIST}
    DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib
)

# install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch config meshes urdf DESTINATION share/${PROJECT_NAME} )#${CATKIN_PACKAGE_SHARE_DESTINATION})

ament_package()
