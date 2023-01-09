//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================
#ifndef GAZEBO_SENSOR_COLLECTION_GAZEBO_ROS_GPS_WITH_DROPOUT_H
#define GAZEBO_SENSOR_COLLECTION_GAZEBO_ROS_GPS_WITH_DROPOUT_H

#include <gazebo/common/Plugin.hh>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <gazebo_sensor_collection/srv/set_reference_geo_pose.hpp>
#include <gazebo_sensor_collection/msg/gps_data.hpp>
#include <gazebo_sensor_collection/sensor_model.h>
#include <gazebo_sensor_collection/update_timer.h>
#include <gazebo_sensor_collection/gnss_config.h>

#include <gazebo_ros/node.hpp>

namespace gazebo{
class GazeboRosGpsWithGeofencing : public ModelPlugin{
public:
     GazeboRosGpsWithGeofencing();
     virtual ~GazeboRosGpsWithGeofencing();

protected:
     virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
     virtual void Reset();
     virtual void Update();

     // void dynamicReconfigureCallback(GNSSConfig &config, uint32_t level);
     rcl_interfaces::msg::SetParametersResult parametersChangedCallback(const std::vector<rclcpp::Parameter> & parameters);

private:
     FieldNoiseCharacteristics field_noise_characteristics;
     std::vector<PeriodicNoiseBox*> noiseBoxes;

     physics::WorldPtr world;     /// \brief The parent World
     physics::LinkPtr link;       /// \brief The link referred to by this plugin

     sensor_msgs::msg::NavSatFix fix_;
     gazebo_sensor_collection::msg::GpsData gps_m;
     geometry_msgs::msg::Vector3Stamped velocity_;
     visualization_msgs::msg::Marker marker_;
     visualization_msgs::msg::Marker pos_marker_;
     visualization_msgs::msg::MarkerArray markerArray_;

     // ros::NodeHandle* node_handle_;
     // ros::Publisher fix_publisher_;
     // ros::Publisher velocity_publisher_;
     // ros::Publisher data_publisher_;
     // ros::Publisher viz_array_pub_;

     gazebo_ros::Node::SharedPtr node_;
     rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_publisher_;
     rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr velocity_publisher_;
     rclcpp::Publisher<gazebo_sensor_collection::msg::GpsData>::SharedPtr data_publisher_;
     rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_array_pub_;
     rclcpp::Service<gazebo_sensor_collection::srv::SetReferenceGeoPose>::SharedPtr set_geopose_srv_;

     std::string namespace_;
     std::string link_name_;
     std::string frame_id_;
     std::string fix_topic_;
     std::string velocity_topic_;
     std::string data_topic_;
     std::string viz_markers_topic_;

     double reference_latitude_;
     double reference_longitude_;
     double reference_heading_;
     double reference_altitude_;
     double radius_north_;
     double radius_east_;

     SensorModel3 position_error_model_;
     SensorModel3 velocity_error_model_;
     UpdateTimer updateTimer;
     event::ConnectionPtr updateConnection;

     // boost::shared_ptr<dynamic_reconfigure::Server<SensorModelConfig> > dynamic_reconfigure_server_position_, dynamic_reconfigure_server_velocity_;
     // boost::shared_ptr<dynamic_reconfigure::Server<GNSSConfig> > dynamic_reconfigure_server_status_;

     /// \brief Callback for the set_spherical_coordinates service
     std::shared_ptr<GNSSConfig> gnss_config;
     rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
     void setGeoposeCb(
          gazebo_sensor_collection::srv::SetReferenceGeoPose::Request::SharedPtr request,
          gazebo_sensor_collection::srv::SetReferenceGeoPose::Response::SharedPtr
     );
     rcl_interfaces::msg::SetParametersResult checkStatusParameters(const std::vector<rclcpp::Parameter> & parameters);

     int measurementCount;
     // ros::NodeHandle private_nh_;
     // ros::NodeHandle nh_;
};

} // namespace gazebo

#endif // GAZEBO_SENSOR_COLLECTION_GAZEBO_ROS_GPS_WITH_DROPOUT_H
