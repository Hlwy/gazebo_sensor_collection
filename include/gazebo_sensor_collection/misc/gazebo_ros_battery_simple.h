#ifndef GAZEBO_SENSOR_COLLECTION_GAZEBO_ROS_BATTERY_SIMPLE_H_
#define GAZEBO_SENSOR_COLLECTION_GAZEBO_ROS_BATTERY_SIMPLE_H_

#include <gazebo/common/Plugin.hh>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <gazebo_ros/node.hpp>
// #include <sensor_msgs/msg/nav_sat_fix.hpp>
// #include <sensor_msgs/msg/nav_sat_status.hpp>
// #include <geometry_msgs/msg/vector3_stamped.hpp>
#include <gazebo_sensor_collection/update_timer.h>
#include <gazebo_sensor_collection/battery_config.h>
#include <gazebo_sensor_collection/msg/battery_data.h>
// #include <gazebo_sensor_collection/srv/set_reference_geo_pose.hpp>

#define SECS_TO_HRS (1.0 / 3600.0)

namespace gazebo{

class SimpleBatteryPlugin : public ModelPlugin{
public:
     SimpleBatteryPlugin();
     virtual ~SimpleBatteryPlugin();

private:
     common::Time getWorldTimeNow(){
          common::Time sim_time;
          #if(GAZEBO_MAJOR_VERSION >= 8)
          sim_time = world->SimTime();
          #else
          sim_time = world->GetSimTime();
          #endif
          return sim_time;
     }
     double getUpdateTimeDelta(){
          double dt;
          #if(GAZEBO_MAJOR_VERSION >= 8)
          dt = updateTimer.getTimeSinceLastUpdate().Double();
          #else
          dt = updateTimer.getTimeSinceLastUpdate().Double();
          #endif
          return dt;
     }
     rclcpp::Time gazeboTimeToRclTime(common::Time sim_time){
          return rclcpp::Time(sim_time.sec, sim_time.nsec);
     }
protected:
     virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
     virtual void Reset();
     virtual void Update();

     rcl_interfaces::msg::SetParametersResult parametersChangedCallback(const std::vector<rclcpp::Parameter> & parameters);

private:
     boost::mutex lock;

     physics::WorldPtr world;  /// \brief The parent World
     physics::LinkPtr link;    /// \brief The link referred to by this plugin

     /// \brief pointer to ros node
     gazebo_ros::Node::SharedPtr node_;
     gazebo_sensor_collection::msg::BatteryData battery_data_;
     rclcpp::Publisher<gazebo_sensor_collection::msg::BatteryData>::SharedPtr battery_pub_;

     UpdateTimer updateTimer;
     event::ConnectionPtr updateConnection;
     std::shared_ptr<SimulatedBatteryConfig> battery_config;
     rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr callback_handle_;

     // void setChargeState(
     //      gazebo_sensor_collection::srv::SetReferenceGeoPose::Request::SharedPtr request,
     //      gazebo_sensor_collection::srv::SetReferenceGeoPose::Response::SharedPtr
     // );
     rcl_interfaces::msg::SetParametersResult checkStatusParameters(const std::vector<rclcpp::Parameter> & parameters);

     std::string namespace_;   /// \brief for setting ROS name space
     std::string link_name_;   /// \brief store link name
     std::string frame_id_;    /// \brief frame id -- could replace old --> battery_link_
     std::string topic_;       /// \brief topic name

     int update_count_;
     double update_rate_;

     double nCells_;
     double vCutoff_;
     double vMin_;
     double vMax_;
     double vInit_;
     double iDraw_;
     double e0_;
     double e1_;
     double capacity_;
     double q0_;
     double tau_;
     bool charging_;
     double chargeRate_;
     double dischargeRate_;
     double rInternal_;

     // Voltage parameters: E(t) = e0 + e1* Q(t)/c
     double curVolts_;
     double prevVolts_;
     double curCharge_; // Instantaneous battery charge in Ah
     double prevCharge_; // Previous battery charge in Ah

     // // deprecated stuff:
     // sdf::ElementPtr sdf;
     // physics::ModelPtr parent;
     // std::string robot_base_frame_;
     // std::string battery_topic_;
     // ros::Publisher charge_state_;

}; // SimpleBatteryPlugin

}
#endif /** GAZEBO_SENSOR_COLLECTION_GAZEBO_ROS_BATTERY_SIMPLE_H_ */
