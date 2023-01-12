#ifndef CUSTOM_GAZEBO_PLUGINS_SIM_BATTERY_CONFIG_H
#define CUSTOM_GAZEBO_PLUGINS_SIM_BATTERY_CONFIG_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <gazebo_ros/node.hpp>

namespace gazebo{

class SimulatedBatteryConfig{
public:
     SimulatedBatteryConfig(gazebo_ros::Node::SharedPtr node){
          this->node_ = node;
          // Initialize the parameters
          node_->declare_parameter<double>("cell_count", 3.0);                  // nCells_ - NumLipo Cells
          node_->declare_parameter<double>("voltage_cutoff", 9.0);              // vCutoff_ - Volts
          node_->declare_parameter<double>("voltage_min", 11.1);                // vMin_ - Volts
          node_->declare_parameter<double>("voltage_max", 12.6);                // vMax_ - Volts
          node_->declare_parameter<double>("voltage_initial", 11.8);            // vInit_ - Volts
          node_->declare_parameter<double>("current_draw", 1.5);                // iDraw_ - Amps
          // Voltage parameters: E(t) = e0 + e1* Q(t)/c
          node_->declare_parameter<double>("e_constant_coeff", 12.694);         // e0_
          node_->declare_parameter<double>("e_linear_coeff", -100.1424);        // e1_
          node_->declare_parameter<double>("charge_capacity", 10.0);            // capacity_ - Battery charge capacity in Ah
          node_->declare_parameter<double>("charge_initial", 10.0);             // q0_ - Initial battery charge in Ah
          node_->declare_parameter<double>("charge_tau", 1.9499);               // tau_ - Current low-pass filter characteristic time in seconds
          // NOT IMPLEMENTED - Parameters
          node_->declare_parameter<bool>("charging", true);                     // charging_ - Flag true when battery is being charged otherwise battery is discharging
          node_->declare_parameter<double>("charge_rate", 0.2);                 // chargeRate_ - Battery Charge rate in A --- NOT_IMPLEMENTED
          node_->declare_parameter<double>("discharge_rate", 0.5);              // dischargeRate_ - Battery Discharge rate in A --- NOT_IMPLEMENTED
          node_->declare_parameter<double>("internal_resistance", 0.061523);    // rInternal_ - Battery internal resistance in Ohm --- NOT_IMPLEMENTED
     }
     virtual ~SimulatedBatteryConfig(){ node_ = nullptr; }
private:
     gazebo_ros::Node::SharedPtr node_;
};

} // namespace gazebo
#endif // CUSTOM_GAZEBO_PLUGINS_SIM_BATTERY_CONFIG_H
