#ifndef SENSOR_INCLUDES_H_
#define SENSOR_INCLUDES_H_

/** SECTION:
     ROS-Dependant Includes
*/
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/MapMetaData.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/TimeReference.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/ChannelFloat32.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <camera_info_manager/camera_info_manager.h>

/** SECTION:
     OpenCV Includes
*/
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

/** SECTION:
     Gazebo Includes
*/
#include <gazebo/util/system.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/physics/Entity.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Event.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/Noise.hh>
#include <gazebo/sensors/SensorFactory.hh>
#include <gazebo/sensors/GpuRaySensor.hh>
#include <gazebo/plugins/CameraPlugin.hh>
#include <gazebo/plugins/DepthCameraPlugin.hh>
#include <gazebo/plugins/RayPlugin.hh>
#include <gazebo/plugins/GpuRayPlugin.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/rendering/Distortion.hh>
#include <gazebo/rendering/RenderTypes.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/rendering/RenderEngine.hh>
#include <gazebo/rendering/GpuLaser.hh>
#include <gazebo/gazebo_config.h>

#if GAZEBO_MAJOR_VERSION < 9
     #include <gazebo/math/Vector3.hh>
     #include <gazebo/math/Angle.hh>
     #include <gazebo/math/Pose.hh>
     #include <gazebo/math/Rand.hh>
#else
     #include <ignition/math/Vector3.hh>
     #include <ignition/math/Angle.hh>
     // #include <ignition/math/Pose.hh>
     #include <ignition/math/Rand.hh>
#endif

#include <sdf/sdf.hh>
#include <sdf/Param.hh>

/** SECTION:
     System Includes
*/
#include <string>
#include <vector>
#include <numeric>
#include <limits>
#include <algorithm>
#include <assert.h>
#include <map>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/algorithm/string.hpp>

/** SECTION:
     Local Includes
*/
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <gazebo_plugins/gazebo_ros_camera_utils.h>
#include <gazebo_plugins/PubQueue.h>
#include <gazebo_plugins/GazeboRosOpenniKinectConfig.h>

#include <hector_gazebo_plugins/sensor_model.h>
#include <hector_gazebo_plugins/update_timer.h>
#include <hector_gazebo_plugins/SetBias.h>
#include <hector_gazebo_plugins/GNSSConfig.h>


#endif // SENSOR_INCLUDES_H_
