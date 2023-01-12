#include "../include/misc/gazebo_ros_battery_simple.h"

#include <ros/ros.h>
#include <sdf/sdf.hh>

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

#include "gazebo/physics/physics.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Battery.hh"

#include <algorithm>
#include <assert.h>

#include <geometry_msgs/Twist.h>

namespace gazebo {

SimpleBatteryPlugin::SimpleBatteryPlugin(){}

// Destructor
SimpleBatteryPlugin::~SimpleBatteryPlugin() {
     updateTimer.Disconnect(updateConnection);
}

void SimpleBatteryPlugin::Reset(){
     updateTimer.Reset();
     update_count_ = 0;
     charging_ = false;
     curVolts_ = vInit_;
     curCharge_ = q0_;
     prevVolts_ = curVolts_;
     prevCharge_ = curCharge_;
}

// Load the controller
void SimpleBatteryPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
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
          RCLCPP_FATAL(this->node_->get_logger(), "SimpleBatteryPlugin plugin error: bodyName: %s does not exist\n", link_name_.c_str());
          return;
     }

     // Define default variables if not defined by user
     frame_id_ = "/world";
     topic_ = "/battery";
     update_rate_ = 10.0;
     // if(_sdf->HasElement("referenceLongitude")) _sdf->GetElement("referenceLongitude")->GetValue()->Get(reference_longitude_);
     // if(_sdf->HasElement("referenceLongitude")) _sdf->GetElement("referenceLongitude")->GetValue()->Get(reference_longitude_);
     // if(_sdf->HasElement("referenceLongitude")) _sdf->GetElement("referenceLongitude")->GetValue()->Get(reference_longitude_);

     nCells_ = 3.0;
     vCutoff_ = 9.0;
     vMin_ = 11.1;
     vMax_ = 12.6;
     vInit_ = 11.8;
     iDraw_ = 1.5;
     e0_ = 12.694;
     e1_ = -100.1424;
     capacity_ = 10.0;
     q0_ = 10.0;
     tau_ = 1.9499;
     charging_ = true;
     chargeRate_ = 0.2;
     dischargeRate_ = 0.5;
     rInternal_ = 0.061523;

     // if(_sdf->HasElement("referenceLongitude")) _sdf->GetElement("referenceLongitude")->GetValue()->Get(reference_longitude_);
     // if(_sdf->HasElement("referenceLongitude")) _sdf->GetElement("referenceLongitude")->GetValue()->Get(reference_longitude_);
     // if(_sdf->HasElement("referenceLongitude")) _sdf->GetElement("referenceLongitude")->GetValue()->Get(reference_longitude_);
     // if(_sdf->HasElement("referenceLongitude")) _sdf->GetElement("referenceLongitude")->GetValue()->Get(reference_longitude_);
     // if(_sdf->HasElement("referenceLongitude")) _sdf->GetElement("referenceLongitude")->GetValue()->Get(reference_longitude_);
     // if(_sdf->HasElement("referenceLongitude")) _sdf->GetElement("referenceLongitude")->GetValue()->Get(reference_longitude_);
     // if(_sdf->HasElement("referenceLongitude")) _sdf->GetElement("referenceLongitude")->GetValue()->Get(reference_longitude_);
     // if(_sdf->HasElement("referenceLongitude")) _sdf->GetElement("referenceLongitude")->GetValue()->Get(reference_longitude_);
     // if(_sdf->HasElement("referenceLongitude")) _sdf->GetElement("referenceLongitude")->GetValue()->Get(reference_longitude_);
     // if(_sdf->HasElement("referenceLongitude")) _sdf->GetElement("referenceLongitude")->GetValue()->Get(reference_longitude_);
     // if(_sdf->HasElement("referenceLongitude")) _sdf->GetElement("referenceLongitude")->GetValue()->Get(reference_longitude_);
     // if(_sdf->HasElement("referenceLongitude")) _sdf->GetElement("referenceLongitude")->GetValue()->Get(reference_longitude_);
     // if(_sdf->HasElement("referenceLongitude")) _sdf->GetElement("referenceLongitude")->GetValue()->Get(reference_longitude_);
     // if(_sdf->HasElement("referenceLongitude")) _sdf->GetElement("referenceLongitude")->GetValue()->Get(reference_longitude_);
     // if(_sdf->HasElement("referenceLongitude")) _sdf->GetElement("referenceLongitude")->GetValue()->Get(reference_longitude_);
     // if(_sdf->HasElement("referenceLongitude")) _sdf->GetElement("referenceLongitude")->GetValue()->Get(reference_longitude_);
     // if(_sdf->HasElement("referenceLongitude")) _sdf->GetElement("referenceLongitude")->GetValue()->Get(reference_longitude_);
     // if(_sdf->HasElement("referenceLongitude")) _sdf->GetElement("referenceLongitude")->GetValue()->Get(reference_longitude_);

     // Make sure the ROS node for Gazebo has already been initialized
     if(!rclcpp::ok()){
          RCLCPP_FATAL_STREAM(node_->get_logger(),
               "A ROS node for Gazebo has not been initialized, unable to load plugin. " <<
               "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)"
          );
          return;
     }

     node_ = new ros::NodeHandle(namespace_);
     battery_pub_ = node_->create_publisher<gazebo_sensor_collection::msg::BatteryData>(topic_, 10);
     // set_geopose_srv_ = node_->create_service<gazebo_sensor_collection::srv::SetReferenceGeoPose>(fix_topic_ + "/set_reference_geopose",
     //     std::bind(&SimpleBatteryPlugin::setGeoposeCb, this, std::placeholders::_1, std::placeholders::_2)
     // );

     // Setup the BatteryConfig parameters and respective callback handlers
     battery_config = std::make_shared<SimulatedBatteryConfig>(node_);
     callback_handle_ = node_->add_on_set_parameters_callback(std::bind(&SimpleBatteryPlugin::parametersChangedCallback, this, std::placeholders::_1));

     Reset();

     // connect Update function
     updateTimer.setUpdateRate(update_rate_);
     updateTimer.Load(world, _sdf);
     updateConnection = updateTimer.Connect(boost::bind(&SimpleBatteryPlugin::Update, this));

     // ROS_GREEN_STREAM("Created a battery");
}

void SimpleBatteryPlugin::Update(){
     common::Time sim_time_now = getWorldTimeNow();
     double dt = getUpdateTimeDelta();

     double k = dt / tau_;
     curCharge_ = prevCharge_ - (dt * k)*SECS_TO_HRS;
     curVolts_ = e0_ + e1_ * (1 - curCharge_ / capacity_);

     // double totalpower = 0.0;
     // current = power(Watts)/Voltage
     // this->idraw = totalpower / _battery->Voltage();

     // ROS_INFO_STREAM(curVolts_);
     // gzdbg << "Current charge:" << curCharge_ << ", at:" << this->sim_time_now << "\n";
     // gzdbg << "Current voltage:" << curVolts_ << ", at:" << this->last_update_time_.Double() << "\n";

     update_count_++;
     prevCharge_ = curCharge_;
     prevVolts_ = curVolts_;

     // battery_data_.header.frame_id = frame_id_;
     battery_data_.stamp = gazeboTimeToRclTime(sim_time_now);
     battery_data_.vmin = vMin_;
     battery_data_.vmax = vMax_;
     battery_data_.voltage = curVolts_;
     battery_pub_->publish(battery_data_);
}

rcl_interfaces::msg::SetParametersResult SimpleBatteryPlugin::checkStatusParameters(const std::vector<rclcpp::Parameter> & parameters){
     using gazebo_sensor_collection::msg::SimulatedBatteryConfig;
     rcl_interfaces::msg::SetParametersResult result;
     result.successful = true;
     result.reason = "";

     // result.successful = false;
     // result.reason = "Parameter not found";
     for (const auto & parameter : parameters){
          std::string name = parameter.get_name();
          if(  name == "cell_count" || name == "voltage_cutoff" ||
               name == "voltage_min" || name == "voltage_max" ||
               name == "voltage_initial" || name == "current_draw" ||
               name == "e_constant_coeff" || name == "e_linear_coeff" ||
               name == "charge_capacity" || name == "charge_initial" ||
               name == "charge_tau" || name == "charging" || name == "charge_rate" ||
               name == "discharge_rate" || name == "internal_resistance"
          ){
               result.successful = true;
               result.reason = "";
               double cell_count(3.0);
               double voltage_cutoff(9.0);
               double voltage_min(11.1);
               double voltage_max(12.6);
               double voltage_initial(11.8);
               double current_draw(1.5);
               double e_constant_coeff(12.694);
               double e_linear_coeff(-100.1424);
               double charge_capacity(10.0);
               double charge_initial(10.0);
               double charge_tau(1.9499);
               bool charging(false);
               double charge_rate(0.2);
               double discharge_rate(0.5);
               double internal_resistance(0.061523);

               node_->get_parameter("cell_count", cell_count);
               node_->get_parameter("voltage_cutoff", voltage_cutoff);
               node_->get_parameter("voltage_min", voltage_min);
               node_->get_parameter("voltage_max", voltage_max);
               node_->get_parameter("voltage_initial", voltage_initial);
               node_->get_parameter("current_draw", current_draw);
               node_->get_parameter("e_constant_coeff", e_constant_coeff);
               node_->get_parameter("e_linear_coeff", e_linear_coeff);
               node_->get_parameter("charge_capacity", charge_capacity);
               node_->get_parameter("charge_initial", charge_initial);
               node_->get_parameter("charge_tau", charge_tau);
               node_->get_parameter("charging", charging);
               node_->get_parameter("charge_rate", charge_rate);
               node_->get_parameter("discharge_rate", discharge_rate);
               node_->get_parameter("internal_resistance", internal_resistance);
               RCLCPP_INFO(node_->get_logger(), "Battery Config parameter changed");

               if(cell_count != nCells_){
                    RCLCPP_WARN(node_->get_logger(), "SimulatedBattery Param \'%s\' changed from %d ---> %d", "nCells_", nCells_, cell_count);
                    nCells_ = cell_count;
               }
               if(voltage_cutoff != vCutoff_){
                    RCLCPP_WARN(node_->get_logger(), "SimulatedBattery Param \'%s\' changed from %d ---> %d", "vCutoff_", vCutoff_, voltage_cutoff);
                    vCutoff_ = voltage_cutoff;
               }
               if(voltage_min != vMin_){
                    RCLCPP_WARN(node_->get_logger(), "SimulatedBattery Param \'%s\' changed from %d ---> %d", "vMin_", vMin_, voltage_min);
                    vMin_ = voltage_min;
               }
               if(voltage_max != vMax_){
                    RCLCPP_WARN(node_->get_logger(), "SimulatedBattery Param \'%s\' changed from %d ---> %d", "vMax_", vMax_, voltage_max);
                    vMax_ = voltage_max;
               }
               if(voltage_initial != vInit_){
                    RCLCPP_WARN(node_->get_logger(), "SimulatedBattery Param \'%s\' changed from %d ---> %d", "vInit_", vInit_, voltage_initial);
                    vInit_ = voltage_initial;
               }
               if(current_draw != iDraw_){
                    RCLCPP_WARN(node_->get_logger(), "SimulatedBattery Param \'%s\' changed from %d ---> %d", "iDraw_", iDraw_, current_draw);
                    iDraw_ = current_draw;
               }
               if(e_constant_coeff != e0_){
                    RCLCPP_WARN(node_->get_logger(), "SimulatedBattery Param \'%s\' changed from %d ---> %d", "e0_", e0_, e_constant_coeff);
                    e0_ = e_constant_coeff;
               }
               if(e_linear_coeff != e1_){
                    RCLCPP_WARN(node_->get_logger(), "SimulatedBattery Param \'%s\' changed from %d ---> %d", "e1_", e1_, e_linear_coeff);
                    e1_ = e_linear_coeff;
               }
               if(charge_capacity != capacity_){
                    RCLCPP_WARN(node_->get_logger(), "SimulatedBattery Param \'%s\' changed from %d ---> %d", "capacity_", capacity_, charge_capacity);
                    capacity_ = charge_capacity;
               }
               if(charge_initial != q0_){
                    RCLCPP_WARN(node_->get_logger(), "SimulatedBattery Param \'%s\' changed from %d ---> %d", "q0_", q0_, charge_initial);
                    q0_ = charge_initial;
               }
               if(charge_tau != tau_){
                    RCLCPP_WARN(node_->get_logger(), "SimulatedBattery Param \'%s\' changed from %d ---> %d", "tau_", tau_, charge_tau);
                    tau_ = charge_tau;
               }

               if(charging != charging_){
                    RCLCPP_WARN(node_->get_logger(), "SimulatedBattery Param \'%s\' changed from %s ---> %s", "charging_", charging_ ? "Charging" : "Discharging" , charging ? "Charging" : "Discharging" );
                    charging_ = charging;
               }
               if(charge_rate != chargeRate_){
                    RCLCPP_WARN(node_->get_logger(), "SimulatedBattery Param \'%s\' changed from %d ---> %d", "chargeRate_", chargeRate_, charge_rate);
                    chargeRate_ = charge_rate;
               }
               if(discharge_rate != dischargeRate_){
                    RCLCPP_WARN(node_->get_logger(), "SimulatedBattery Param \'%s\' changed from %d ---> %d", "dischargeRate_", dischargeRate_, discharge_rate);
                    dischargeRate_ = discharge_rate;
               }
               if(internal_resistance != rInternal_){
                    RCLCPP_WARN(node_->get_logger(), "SimulatedBattery Param \'%s\' changed from %d ---> %d", "rInternal_", rInternal_, internal_resistance);
                    rInternal_ = internal_resistance;
               }

          }
     }
     return result;
}
rcl_interfaces::msg::SetParametersResult SimpleBatteryPlugin::parametersChangedCallback(const std::vector<rclcpp::Parameter> & parameters){
     rcl_interfaces::msg::SetParametersResult result_status, result_pos, result_vel;

     result_status = checkStatusParameters(parameters);
     // result_pos = position_error_model_.parametersChangedCallback(parameters);
     // result_vel = velocity_error_model_.parametersChangedCallback(parameters);

     // Return the final result
     if(result_status.successful
          // && result_pos.successful && result_vel.successful
     ){
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          result.reason = "";
          return result;
     } else{
          if(!result_status.successful){ return result_status; }
          // else if(!result_pos.successful){ return result_pos; }
          // else{ return result_vel; }
     }
}

GZ_REGISTER_MODEL_PLUGIN(SimpleBatteryPlugin)
}
