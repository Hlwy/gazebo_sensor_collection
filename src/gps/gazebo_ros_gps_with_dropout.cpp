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

#include <gazebo_sensor_collection/gps/gazebo_ros_gps_with_dropout.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/SphericalCoordinates.hh>

#include <tf2/LinearMath/Transform.h>
#include <iostream>

// WGS84 constants
static const double equatorial_radius = 6378137.0;
static const double flattening = 1.0/298.257223563;
static const double excentrity2 = 2*flattening - flattening*flattening;

// default reference position
static const double DEFAULT_REFERENCE_LATITUDE  = 49.9;
static const double DEFAULT_REFERENCE_LONGITUDE = 8.9;
static const double DEFAULT_REFERENCE_HEADING   = 0.0;
static const double DEFAULT_REFERENCE_ALTITUDE  = 0.0;

// static std::ostringstream stream;
// static int ros_quick_info_count = 0;
// #define ROS_QUICK_INFO(x) stream << ros_quick_info_count++ << ". " << x << std::endl; ROS_INFO(stream.str().c_str())

namespace gazebo {

GazeboRosGpsWithGeofencing::GazeboRosGpsWithGeofencing(){}
GazeboRosGpsWithGeofencing::~GazeboRosGpsWithGeofencing(){
     updateTimer.Disconnect(updateConnection);
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosGpsWithGeofencing::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
     int i = 0;
     int j = 0;
     PeriodicNoiseBox* box = nullptr;
     sdf::ElementPtr p = nullptr;
     bool success = false;

     BiomassDensityGPSNoiseCoefficient* density_char;
     BiomassHeightGPSNoiseCoefficient* height_char;
     world = _model->GetWorld();
     node_ = gazebo_ros::Node::Get(_sdf);

     // load parameters
     if(!_sdf->HasElement("robotNamespace")) namespace_.clear();
     else namespace_ = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString();

     if(!_sdf->HasElement("bodyName")){
          link = _model->GetLink();
          link_name_ = link->GetName();
     } else{
          link_name_ = _sdf->GetElement("bodyName")->GetValue()->GetAsString();
          link = _model->GetLink(link_name_);
     }

     if(!link){
          RCLCPP_FATAL(this->node_->get_logger(), "GazeboRosGpsWithGeofencing plugin error: bodyName: %s does not exist\n", link_name_.c_str());
          return;
     }

     RCLCPP_INFO(this->node_->get_logger(), "Configuring GPS noise and dropout behavior...");
     // while(p = _sdf->GetNextElement() && i < 100){
     //      ROS_QUICK_INFO("" << p->GetName());
     //      i++;
     // }

     if(_sdf->HasElement("field_gps_noise_characteristics")){
          for(i=0;i<99;i++){
               p = _sdf->GetNextElement("biomass_density_noise_coefficient");
               if(!p){ break; }

               density_char = new BiomassDensityGPSNoiseCoefficient;
               success = p->GetElement("r1")->GetValue()->Get(density_char->density_range.min);
               success = p->GetElement("r2")->GetValue()->Get(density_char->density_range.max);
               success = p->GetElement("mean")->GetValue()->Get(density_char->mean);
               success = p->GetElement("variance")->GetValue()->Get(density_char->variance);
               this->field_noise_characteristics.biomass_density_gps_noise_coefficients.push_back(density_char);
          }
          for(i=0;i<99;i++){
               p = _sdf->GetNextElement("biomass_height_noise_coefficient");
               if(!p){ break; }

               height_char = new BiomassHeightGPSNoiseCoefficient;
               success = p->GetElement("r1")->GetValue()->Get(height_char->height_range.min);
               success = p->GetElement("r2")->GetValue()->Get(height_char->height_range.max);
               success = p->GetElement("mean")->GetValue()->Get(height_char->mean);
               success = p->GetElement("variance")->GetValue()->Get(height_char->variance);
               this->field_noise_characteristics.biomass_height_gps_noise_coefficients.push_back(height_char);
               RCLCPP_INFO_STREAM(this->node_->get_logger(), "Height coefficient: " << i << " \n Range, mean, variance ... [" << height_char->height_range.min << ", " << height_char->height_range.max << "] , " << height_char->mean << ", " << height_char->variance);
          }
     } else{ RCLCPP_INFO(this->node_->get_logger(), "No GPS noise characteristics found..."); }

     RCLCPP_INFO(this->node_->get_logger(), "Loading periodic noise box objects...");
     for(i=0;i<99;i++){
          p = _sdf->GetNextElement("periodic_noise_box");
          if(!p){ break; }

          box = new PeriodicNoiseBox;
          success = p->GetElement("cx")->GetValue()->Get(box->center.cartesian.x);
          success = p->GetElement("cy")->GetValue()->Get(box->center.cartesian.y);
          success = p->GetElement("cz")->GetValue()->Get(box->center.cartesian.z);
          success = p->GetElement("w")->GetValue()->Get(box->dimensions.width);
          success = p->GetElement("l")->GetValue()->Get(box->dimensions.length);
          success = p->GetElement("h")->GetValue()->Get(box->dimensions.height);
          success = p->GetElement("rot")->GetValue()->Get(box->rotation.xy_rotation);
          success = p->GetElement("nps")->GetValue()->Get(box->static_parameters.noise_period_s );
          success = p->GetElement("nmpm")->GetValue()->Get(box->static_parameters.noise_mean_periodic_mean);
          success = p->GetElement("nmpv")->GetValue()->Get(box->static_parameters.noise_mean_periodic_variance);
          success = p->GetElement("nvpm")->GetValue()->Get(box->static_parameters.noise_variance_periodic_mean);
          success = p->GetElement("nvpv")->GetValue()->Get(box->static_parameters.noise_variance_perioidic_variance);
          this->noiseBoxes.push_back(box);
     }

     RCLCPP_INFO(this->node_->get_logger(), "Loading field plot noise box objects...");

     for(i=0;i<99;i++){
          double temp1, temp2;
          p = _sdf->GetNextElement("plot");
          if(!p){ break; }

          box = new PeriodicNoiseBox;
          success = p->GetElement("x")->GetValue()->Get(box->center.cartesian.x);
          success = p->GetElement("y")->GetValue()->Get(box->center.cartesian.y);
          success = p->GetElement("z")->GetValue()->Get(box->center.cartesian.z);
          success = p->GetElement("row_width")->GetValue()->Get(temp1);
          success = p->GetElement("row_count")->GetValue()->Get(temp2);
          box->dimensions.width = temp1 * (temp2-1);
          success = p->GetElement("row_length")->GetValue()->Get(box->dimensions.length);
          success = p->GetElement("average_height")->GetValue()->Get(box->dimensions.height);
          success = p->GetElement("rotation")->GetValue()->Get(box->rotation.xy_rotation);
          box->static_parameters.noise_period_s = -1;
          box->static_parameters.noise_mean_periodic_mean = -1;
          box->static_parameters.noise_mean_periodic_variance = -1;
          box->static_parameters.noise_variance_periodic_mean = -1;
          box->static_parameters.noise_variance_perioidic_variance = -1;

          float m = 0;
          float v = 0;
          for(j=0;j<this->field_noise_characteristics.biomass_height_gps_noise_coefficients.size();j++){
               height_char = this->field_noise_characteristics.biomass_height_gps_noise_coefficients[j];
               if(box->dimensions.height >= height_char->height_range.min && box->dimensions.height <= height_char->height_range.max){
                    m = height_char->mean;
                    v = height_char->variance;
               }
          }
          box->dynamic_parameters.noise_mean = m;
          box->dynamic_parameters.noise_variance = v;
          this->noiseBoxes.push_back(box);

          RCLCPP_INFO_STREAM(this->node_->get_logger(), "GPS noise box for plot: " << i \
               << " \n [x, y, z], [width, length, height], rotation, [noise-mean, noise-variance] ...\n" \
               << "[ " << box->center.cartesian.x << "," << box->center.cartesian.y << "," << box->center.cartesian.z << "]\n" \
               << "[ " << box->dimensions.width << "," << box->dimensions.length << "," << box->dimensions.height << "]\n" \
               << box->rotation.xy_rotation << "\n" \
               << "[ " << box->dynamic_parameters.noise_mean << "," << box->dynamic_parameters.noise_variance << "]\n");
     }

     //   height_char = new BiomassHeightGPSNoiseCoefficient;
     //   height_char->height_range.min = 0; height_char->height_range.max = .2;     height_char->mean = 0;   height_char->variance = 1;
     //   this->field_noise_characteristics.biomass_height_gps_noise_coefficients.push_back(height_char);
     //   height_char = new BiomassHeightGPSNoiseCoefficient;
     //   height_char->height_range.min = .2;     height_char->height_range.max = .5;     height_char->mean = 0;   height_char->variance = 1.2;
     //   this->field_noise_characteristics.biomass_height_gps_noise_coefficients.push_back(height_char);
     //   height_char = new BiomassHeightGPSNoiseCoefficient;
     //   height_char->height_range.min = .5;     height_char->height_range.max = 1; height_char->mean = 0;   height_char->variance = 1.5;
     //   this->field_noise_characteristics.biomass_height_gps_noise_coefficients.push_back(height_char);
     //   height_char = new BiomassHeightGPSNoiseCoefficient;
     //   height_char->height_range.min = 1; height_char->height_range.max = 2; height_char->mean = 0;   height_char->variance = 4;
     //   this->field_noise_characteristics.biomass_height_gps_noise_coefficients.push_back(height_char);
     //   height_char = new BiomassHeightGPSNoiseCoefficient;
     //   height_char->height_range.min = 2; height_char->height_range.max = -1;     height_char->mean = 0;   height_char->variance = 5;
     //   this->field_noise_characteristics.biomass_height_gps_noise_coefficients.push_back(height_char);

     float pw = .762*3;
     box = new PeriodicNoiseBox;
     box->center.cartesian.x = pw / 2; box->center.cartesian.y = 5; box->center.cartesian.z = -1.5;
     box->dimensions.height = 7; box->dimensions.length = 10; box->dimensions.width = pw;
     box->dynamic_parameters.noise_mean = 0; box->dynamic_parameters.noise_variance = 4;
     box->static_parameters.noise_period_s = -1;
     box->rotation.xy_rotation = 0;
     this->noiseBoxes.push_back(box);
     box = new PeriodicNoiseBox;
     box->center.cartesian.x = pw / 2 + 2 + pw; box->center.cartesian.y = 5; box->center.cartesian.z = -1.5;
     box->dimensions.height = 7; box->dimensions.length = 10; box->dimensions.width = pw;
     box->dynamic_parameters.noise_mean = 0; box->dynamic_parameters.noise_variance = 4;
     box->static_parameters.noise_period_s = -1;
     box->rotation.xy_rotation = 0;
     this->noiseBoxes.push_back(box);
     box = new PeriodicNoiseBox;
     box->center.cartesian.x = pw / 2; box->center.cartesian.y = 1; box->center.cartesian.z = -1.5;
     box->dimensions.height = 7; box->dimensions.length = 10; box->dimensions.width = pw;
     box->dynamic_parameters.noise_mean = 0; box->dynamic_parameters.noise_variance = 4;
     box->static_parameters.noise_period_s = -1;
     box->rotation.xy_rotation = 0;
     this->noiseBoxes.push_back(box);
     box = new PeriodicNoiseBox;
     box->center.cartesian.x =  pw / 2 + 2 + pw; box->center.cartesian.y = 1; box->center.cartesian.z = -3.1;
     box->dimensions.height = 7; box->dimensions.length = 10; box->dimensions.width = pw;
     box->dynamic_parameters.noise_mean = 0; box->dynamic_parameters.noise_variance = 1.2;
     box->static_parameters.noise_period_s = -1;
     box->rotation.xy_rotation = 0;
     this->noiseBoxes.push_back(box);

     // default parameters
     frame_id_ = "/world";
     fix_topic_ = "/gps/fix";
     velocity_topic_ = "/gps/fix_velocity";
     data_topic_ = "/gps/data";
     viz_markers_topic_ = "/viz_gps_pts";
     reference_latitude_  = DEFAULT_REFERENCE_LATITUDE;
     reference_longitude_ = DEFAULT_REFERENCE_LONGITUDE;
     reference_heading_   = DEFAULT_REFERENCE_HEADING * M_PI/180.0;
     reference_altitude_  = DEFAULT_REFERENCE_ALTITUDE;

     fix_.status.status  = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
     fix_.status.service = 0;

     if(_sdf->HasElement("frameId")) frame_id_ = _sdf->GetElement("frameId")->GetValue()->GetAsString();
     if(_sdf->HasElement("topicName")) fix_topic_ = _sdf->GetElement("topicName")->GetValue()->GetAsString();
     if(_sdf->HasElement("dataTopicName")) data_topic_ = _sdf->GetElement("dataTopicName")->GetValue()->GetAsString();
     if(_sdf->HasElement("velocityTopicName")) velocity_topic_ = _sdf->GetElement("velocityTopicName")->GetValue()->GetAsString();
     if(_sdf->HasElement("vizMarkersTopicName")) viz_markers_topic_ = _sdf->GetElement("vizMarkersTopicName")->GetValue()->GetAsString();
     if(_sdf->HasElement("referenceLatitude")) _sdf->GetElement("referenceLatitude")->GetValue()->Get(reference_latitude_);
     if(_sdf->HasElement("referenceLongitude")) _sdf->GetElement("referenceLongitude")->GetValue()->Get(reference_longitude_);
     if(_sdf->HasElement("referenceAltitude")) _sdf->GetElement("referenceAltitude")->GetValue()->Get(reference_altitude_);
     if(_sdf->HasElement("referenceHeading")){
          if(_sdf->GetElement("referenceHeading")->GetValue()->Get(reference_heading_)){
               reference_heading_ *= M_PI/180.0;
          }
     }

     if(_sdf->HasElement("status")){
          int status = fix_.status.status;
          if(_sdf->GetElement("status")->GetValue()->Get(status)){
               fix_.status.status = static_cast<sensor_msgs::msg::NavSatStatus::_status_type>(status);
          }
     }
     if(_sdf->HasElement("service")){
          unsigned int service = fix_.status.service;
          if(_sdf->GetElement("service")->GetValue()->Get(service)){
               fix_.status.service = static_cast<sensor_msgs::msg::NavSatStatus::_service_type>(service);
          }
     }

     fix_.header.frame_id = frame_id_;
     velocity_.header.frame_id = frame_id_;

     position_error_model_.Load(node_, _sdf);
     velocity_error_model_.Load(node_, _sdf, "velocity");

     // calculate earth radii
     double temp = 1.0 / (1.0 - excentrity2 * sin(reference_latitude_ * M_PI/180.0) * sin(reference_latitude_ * M_PI/180.0));
     double prime_vertical_radius = equatorial_radius * sqrt(temp);
     radius_north_ = prime_vertical_radius * (1 - excentrity2) * temp;
     radius_east_  = prime_vertical_radius * cos(reference_latitude_ * M_PI/180.0);

     // Make sure the ROS node for Gazebo has already been initialized
     if(!rclcpp::ok()){
          RCLCPP_FATAL_STREAM(node_->get_logger(),
               "A ROS node for Gazebo has not been initialized, unable to load plugin. " <<
               "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)"
          );
          return;
     }

     // node_handle_ = new ros::NodeHandle(namespace_);
     // fix_publisher_ = node_handle_->advertise<sensor_msgs::msg::NavSatFix>(fix_topic_, 10);
     // velocity_publisher_ = node_handle_->advertise<geometry_msgs::msg::Vector3Stamped>(velocity_topic_, 10);
     // data_publisher_ = node_handle_->advertise<gazebo_sensor_collection::msg::GpsData>(data_topic_, 10);
     fix_publisher_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>(fix_topic_, 10);
     velocity_publisher_ = node_->create_publisher<geometry_msgs::msg::Vector3Stamped>(velocity_topic_, 10);
     data_publisher_ = node_->create_publisher<gazebo_sensor_collection::msg::GpsData>(data_topic_, 10);
     viz_array_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(viz_markers_topic_, 10);
     set_geopose_srv_ = node_->create_service<gazebo_sensor_collection::srv::SetReferenceGeoPose>(fix_topic_ + "/set_reference_geopose",
          std::bind(&GazeboRosGpsWithGeofencing::setGeoposeCb, this, std::placeholders::_1, std::placeholders::_2)
     );

     this->measurementCount = 0;
     // this->marker_.header.frame_id = "odom";
     this->marker_.header.frame_id = frame_id_;
     this->marker_.header.stamp = node_->get_clock()->now();
     this->marker_.ns = "gps_markers";
     this->marker_.type = visualization_msgs::msg::Marker::ARROW;
     this->marker_.action = visualization_msgs::msg::Marker::ADD;
     this->marker_.pose.orientation.x = 0;
     this->marker_.pose.orientation.y = 0;
     this->marker_.pose.orientation.z = 0;
     this->marker_.pose.orientation.w = 0;
     this->marker_.scale.x = 0.1;
     this->marker_.scale.y = 0.1;
     this->marker_.scale.z = 0.1;

     // this->pos_marker_.header.frame_id = "odom";
     this->pos_marker_.header.frame_id = frame_id_;
     this->pos_marker_.header.stamp = node_->get_clock()->now();
     this->pos_marker_.ns = "pos_markers";
     this->pos_marker_.type = visualization_msgs::msg::Marker::CYLINDER;
     this->pos_marker_.action = visualization_msgs::msg::Marker::ADD;
     this->pos_marker_.pose.orientation.x = 0;
     this->pos_marker_.pose.orientation.y = 0;
     this->pos_marker_.pose.orientation.z = 0;
     this->pos_marker_.pose.orientation.w = 0;
     this->pos_marker_.scale.x = 0.1;
     this->pos_marker_.scale.y = 0.1;
     this->pos_marker_.scale.z = 0.1;

     // Setup the GNSS parameters
     gnss_config = std::make_shared<GNSSConfig>(node_);
     // Setup the parameters handler
     callback_handle_ = node_->add_on_set_parameters_callback(std::bind(&GazeboRosGpsWithGeofencing::parametersChangedCallback, this, std::placeholders::_1));
     // // setup dynamic_reconfigure servers
     // dynamic_reconfigure_server_position_.reset(new dynamic_reconfigure::Server<SensorModelConfig>(ros::NodeHandle(*node_handle_, fix_topic_ + "/position")));
     // dynamic_reconfigure_server_velocity_.reset(new dynamic_reconfigure::Server<SensorModelConfig>(ros::NodeHandle(*node_handle_, fix_topic_ + "/velocity")));
     // dynamic_reconfigure_server_status_.reset(new dynamic_reconfigure::Server<GNSSConfig>(ros::NodeHandle(*node_handle_, fix_topic_ + "/status")));
     // dynamic_reconfigure_server_position_->setCallback(boost::bind(&SensorModel3::dynamicReconfigureCallback, &position_error_model_, _1, _2));
     // dynamic_reconfigure_server_velocity_->setCallback(boost::bind(&SensorModel3::dynamicReconfigureCallback, &velocity_error_model_, _1, _2));
     // dynamic_reconfigure_server_status_->setCallback(boost::bind(&GazeboRosGpsWithGeofencing::dynamicReconfigureCallback, this, _1, _2));

     Reset();

     // connect Update function
     updateTimer.setUpdateRate(4.0);
     updateTimer.Load(world, _sdf);
     updateConnection = updateTimer.Connect(boost::bind(&GazeboRosGpsWithGeofencing::Update, this));
}

void GazeboRosGpsWithGeofencing::setGeoposeCb(
    gazebo_sensor_collection::srv::SetReferenceGeoPose::Request::SharedPtr request,
    gazebo_sensor_collection::srv::SetReferenceGeoPose::Response::SharedPtr)
{
    reference_latitude_ = request->geo_pose.position.latitude;
    reference_longitude_ = request->geo_pose.position.longitude;
    tf2::Quaternion q(
        request->geo_pose.orientation.x,
        request->geo_pose.orientation.y,
        request->geo_pose.orientation.z,
        request->geo_pose.orientation.w
    );

    tf2::Matrix3x3 m(q);
    tf2Scalar yaw, pitch, roll;
    m.getEulerYPR(yaw, pitch, roll);
    reference_heading_ = (M_PI / 2.0) - yaw;
    reference_altitude_ = request->geo_pose.position.altitude;
    Reset();
}
rcl_interfaces::msg::SetParametersResult GazeboRosGpsWithGeofencing::checkStatusParameters(const std::vector<rclcpp::Parameter> & parameters){
    using sensor_msgs::msg::NavSatStatus;
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "";

    // result.successful = false;
    // result.reason = "Parameter not found";
    for (const auto & parameter : parameters){
        std::string name = parameter.get_name();
        if(name == "status_fix" ||
            name == "status_sbas_fix" ||
            name == "status_gbas_fix" ||
            name == "service_gps" ||
            name == "service_glonass" ||
            name == "service_compass" ||
            name == "service_galileo")
        {
            result.successful = true;
            result.reason = "";

            bool status_fix(false);
            bool status_sbas_fix(false);
            bool status_gbas_fix(false);
            bool service_gps(false);
            bool service_glonass(false);
            bool service_compass(false);
            bool service_galileo(false);

            node_->get_parameter("status_fix", status_fix);
            node_->get_parameter("status_sbas_fix", status_sbas_fix);
            node_->get_parameter("status_gbas_fix", status_gbas_fix);
            node_->get_parameter("service_gps", service_gps);
            node_->get_parameter("service_glonass", service_glonass);
            node_->get_parameter("service_compass", service_compass);
            node_->get_parameter("service_galileo", service_galileo);

            RCLCPP_INFO(node_->get_logger(), "Fix status paramter changed");

            if(!status_fix){ fix_.status.status = NavSatStatus::STATUS_NO_FIX; }
            else{ fix_.status.status = (status_sbas_fix ? NavSatStatus::STATUS_SBAS_FIX : 0) | (status_gbas_fix ? NavSatStatus::STATUS_GBAS_FIX : 0); }

            fix_.status.service = (
                (service_gps ? NavSatStatus::SERVICE_GPS : 0) |
                (service_glonass ? NavSatStatus::SERVICE_GLONASS : 0) |
                (service_compass ? NavSatStatus::SERVICE_COMPASS : 0) |
                (service_galileo ? NavSatStatus::SERVICE_GALILEO : 0)
            );
        }
    }
    return result;
}
rcl_interfaces::msg::SetParametersResult GazeboRosGpsWithGeofencing::parametersChangedCallback(const std::vector<rclcpp::Parameter> & parameters){
    rcl_interfaces::msg::SetParametersResult result_status, result_pos, result_vel;

    result_status = checkStatusParameters(parameters);
    result_pos = position_error_model_.parametersChangedCallback(parameters);
    result_vel = velocity_error_model_.parametersChangedCallback(parameters);

    // Return the final result
    if(result_status.successful && result_pos.successful && result_vel.successful){
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "";
        return result;
    } else{
        if(!result_status.successful){ return result_status; }
        else if(!result_pos.successful){ return result_pos; }
        else{ return result_vel; }
    }
}

void GazeboRosGpsWithGeofencing::Reset(){
     updateTimer.Reset();
     position_error_model_.reset();
     velocity_error_model_.reset();
}


////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosGpsWithGeofencing::Update(){
     int i;
     double px, py, pz;
     double vx, vy, vz;
     // common::Time sim_time = world->GetSimTime();
     // double dt = updateTimer.getTimeSinceLastUpdate().Double();
     // math::Pose pose = link->GetWorldPose();

     #if(GAZEBO_MAJOR_VERSION >= 8)
     common::Time sim_time = world->SimTime();
     double dt = updateTimer.getTimeSinceLastUpdate().Double();

     ignition::math::Pose3d pose = link->WorldPose();
     ignition::math::Vector3d velocity = velocity_error_model_(link->WorldLinearVel(), dt);
     ignition::math::Vector3d position = position_error_model_(pose.Pos(), dt);
     px = position.X();
     py = position.Y();
     pz = position.Z();
     vx = velocity.X();
     vy = velocity.Y();
     vz = velocity.Z();
     #else
     common::Time sim_time = world->GetSimTime();
     double dt = updateTimer.getTimeSinceLastUpdate().Double();

     math::Pose pose = link->GetWorldPose();
     gazebo::math::Vector3 velocity = velocity_error_model_(link->GetWorldLinearVel(), dt);
     gazebo::math::Vector3 position = position_error_model_(pose.pos, dt);
     px = position.x;
     py = position.y;
     pz = position.z;
     vx = velocity.x;
     vy = velocity.y;
     vz = velocity.z;
     #endif

     // An offset error in the velocity is integrated into the position error for the next timestep.
     // Note: Usually GNSS receivers have almost no drift in the velocity signal.
     position_error_model_.setCurrentDrift(position_error_model_.getCurrentDrift() + dt * velocity_error_model_.getCurrentDrift());
     fix_.header.stamp = rclcpp::Time(sim_time.sec, sim_time.nsec);
     velocity_.header.stamp = fix_.header.stamp;

     PeriodicNoiseBox* box = nullptr;
     for(i=0;i<this->noiseBoxes.size();i++){
          box = this->noiseBoxes[i];
          if(box->contains((float)px, (float)py, (float)pz)){
               RCLCPP_INFO_STREAM(this->node_->get_logger(), "Contained in noise box " << i << "...( p=[" <<  px << " , " << py << "] )\n");
               RCLCPP_INFO_STREAM(this->node_->get_logger(), "GPS noise box for plot: " << i \
                    << " \n [x, y, z], [width, length, height], rotation, [noise-mean, noise-variance] ...\n" \
                    << "[ " << box->center.cartesian.x << "," << box->center.cartesian.y << "," << box->center.cartesian.z << "]\n" \
                    << "[ " << box->dimensions.width << "," << box->dimensions.length << "," << box->dimensions.height << "]\n" \
                    << box->rotation.xy_rotation << "\n" \
                    << "[ " << box->dynamic_parameters.noise_mean << "," << box->dynamic_parameters.noise_variance << "]\n"
               );
               break;
          }
          box = nullptr;
     }
     if(box){
          #if(GAZEBO_MAJOR_VERSION >= 8)
          position_error_model_.gaussian_noise.X() *= sqrt(box->dynamic_parameters.noise_variance);
          position_error_model_.gaussian_noise.Y() *= sqrt(box->dynamic_parameters.noise_variance);
          position_error_model_.gaussian_noise.Z() *= sqrt(box->dynamic_parameters.noise_variance);
          #else
          position_error_model_.gaussian_noise.x *= sqrt(box->dynamic_parameters.noise_variance);
          position_error_model_.gaussian_noise.y *= sqrt(box->dynamic_parameters.noise_variance);
          position_error_model_.gaussian_noise.z *= sqrt(box->dynamic_parameters.noise_variance);
          #endif
	}

     #if(GAZEBO_MAJOR_VERSION >= 8)
     fix_.latitude  = reference_latitude_  + ( cos(reference_heading_) * position.X() + sin(reference_heading_) * position.Y()) / radius_north_ * 180.0/M_PI;
     fix_.longitude = reference_longitude_ - (-sin(reference_heading_) * position.X() + cos(reference_heading_) * position.Y()) / radius_east_  * 180.0/M_PI;
     fix_.altitude  = reference_altitude_  + position.Z();
     velocity_.vector.x =  cos(reference_heading_) * velocity.X() + sin(reference_heading_) * velocity.Y();
     velocity_.vector.y = -sin(reference_heading_) * velocity.X() + cos(reference_heading_) * velocity.Y();
     velocity_.vector.z = velocity.Z();
     fix_.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
     fix_.position_covariance[0] = position_error_model_.drift.X()*position_error_model_.drift.X() + position_error_model_.gaussian_noise.X()*position_error_model_.gaussian_noise.X();
     fix_.position_covariance[4] = position_error_model_.drift.Y()*position_error_model_.drift.Y() + position_error_model_.gaussian_noise.Y()*position_error_model_.gaussian_noise.Y();
     fix_.position_covariance[8] = position_error_model_.drift.Z()*position_error_model_.drift.Z() + position_error_model_.gaussian_noise.Z()*position_error_model_.gaussian_noise.Z();
     #else
     fix_.latitude  = reference_latitude_  + ( cos(reference_heading_) * position.x + sin(reference_heading_) * position.y) / radius_north_ * 180.0/M_PI;
     fix_.longitude = reference_longitude_ - (-sin(reference_heading_) * position.x + cos(reference_heading_) * position.y) / radius_east_  * 180.0/M_PI;
     fix_.altitude  = reference_altitude_  + position.z;
     velocity_.vector.x =  cos(reference_heading_) * velocity.x + sin(reference_heading_) * velocity.y;
     velocity_.vector.y = -sin(reference_heading_) * velocity.x + cos(reference_heading_) * velocity.y;
     velocity_.vector.z = velocity.z;
     fix_.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
     fix_.position_covariance[0] = position_error_model_.drift.x*position_error_model_.drift.x + position_error_model_.gaussian_noise.x*position_error_model_.gaussian_noise.x;
     fix_.position_covariance[4] = position_error_model_.drift.y*position_error_model_.drift.y + position_error_model_.gaussian_noise.y*position_error_model_.gaussian_noise.y;
     fix_.position_covariance[8] = position_error_model_.drift.z*position_error_model_.drift.z + position_error_model_.gaussian_noise.z*position_error_model_.gaussian_noise.z;
     #endif

     gps_m.stamp = fix_.header.stamp;
     gps_m.latitude = fix_.latitude;
     gps_m.longitude = fix_.longitude;
     gps_m.altitude = fix_.altitude;
     gps_m.reference_latitude = reference_latitude_;
     gps_m.reference_longitude = reference_longitude_;
     gps_m.reference_altitude = reference_altitude_;
     gps_m.reference_heading = reference_heading_;
     gps_m.velocity.x = velocity_.vector.x;
     gps_m.velocity.y = velocity_.vector.y;
     gps_m.velocity.z = velocity_.vector.z;
     gps_m.covariance.x = fix_.position_covariance[0];
     gps_m.covariance.y = fix_.position_covariance[4];
     gps_m.covariance.z = fix_.position_covariance[8];
     gps_m.covariance_type = fix_.position_covariance_type;
     gps_m.service = fix_.status.status;
     gps_m.status = fix_.status.service;

     fix_publisher_->publish(fix_);
     velocity_publisher_->publish(velocity_);
     data_publisher_->publish(gps_m);


     this->marker_.color.a = 1.0;
     this->marker_.pose.position.x = px; this->marker_.pose.position.y = py; this->marker_.pose.position.z = pz;
     this->marker_.color.r = 1.0; this->marker_.color.g = 1.0; this->marker_.color.b = 0.0;
     this->marker_.id = this->measurementCount % 10;
     measurementCount++;
     markerArray_.markers.push_back(this->marker_);
     viz_array_pub_->publish(markerArray_);
     markerArray_.markers.clear();

     this->pos_marker_.color.a = 1.0;
     this->pos_marker_.pose.position.x = px; this->pos_marker_.pose.position.y = py; this->pos_marker_.pose.position.z = pz;
     this->pos_marker_.color.r = 0.0; this->pos_marker_.color.g = 0.0; this->pos_marker_.color.b = 1.0;
     this->pos_marker_.id++;
     markerArray_.markers.push_back(this->pos_marker_);
     viz_array_pub_->publish(markerArray_);
     markerArray_.markers.clear();
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosGpsWithGeofencing)

} // namespace gazebo
