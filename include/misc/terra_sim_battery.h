#ifndef TERRA_ROS_SIM_BATTERY_H_
#define TERRA_ROS_SIM_BATTERY_H_

#include <map>
#include <string>

#include <sdf/sdf.hh>
#include <gazebo/common/common.hh>
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/CommonTypes.hh"
#include "gazebo/physics/physics.hh"

#include <ros/ros.h>
#include "ros/subscribe_options.h"
#include <ros/advertise_options.h>
#include "ros/callback_queue.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "std_msgs/Bool.h"
#include <geometry_msgs/Twist.h>
#include <terrasentia_sensors/TerraBattery.h>

#include "ros_debug.h"

namespace gazebo{

enum power{
    OFF = 0,
    ON = 1
};

class BatteryPlugin : public ModelPlugin{

private:
     boost::mutex lock;
     common::BatteryPtr battery;

     ros::NodeHandle* _nh;

     ros::Publisher battery_level_pub_;
     terrasentia_sensors::TerraBattery sim_battery_;
     // ros::Publisher charge_state_;

     /**  ====================
      SDF definable parameters
     ========================= */

     std::string robot_namespace_;
     std::string robot_base_frame_;
     std::string battery_name_;
     std::string battery_link_;
     std::string battery_topic_;

     double update_rate_;
     double update_period_;

     // Voltage parameters: E(t) = e0 + e1* Q(t)/c
     double et;
     double e0;
     double e1;

     double q0;     // Initial battery charge in Ah
     double qt;     // Charge rate in A
     double q;      // Instantaneous battery charge in Ah

     double c;      // Battery capacity in Ah
     double r;      // Battery inner resistance in Ohm
     double tau;    // Current low-pass filter characteristic time in seconds
     double iraw;   // Raw battery current in A
     double ismooth;// Smoothed battery current in A

     double _vmax;
     double _vmin;
     double _v0;
     double dBattery;

     bool alive_;
     double x_;
     double rot_;
     int update_count_;

     double OnUpdateVoltage(const common::BatteryPtr &_battery);

     common::Time last_update_time_;
     common::Time cur_time_;
     bool charging;

     physics::WorldPtr world;
     physics::ModelPtr parent;
     sdf::ElementPtr sdf;

     ros::CallbackQueue queue_;
     boost::thread callback_queue_thread_;
     void QueueThread();

     event::ConnectionPtr update_connection_;

public:

     BatteryPlugin();
     ~BatteryPlugin();

     // Inherited from ModelPlugin
     virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
     virtual void Init();
     virtual void Reset();

     void publishBatteryInfo();

protected:
     virtual void OnUpdate();

}; // BatteryPlugin

}
#endif /** TERRA_ROS_SIM_BATTERY_H_ */
