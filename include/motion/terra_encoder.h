#ifndef GAZEBO_ROS_ENCODER_H_
#define GAZEBO_ROS_ENCODER_H_

#include <map>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <terrasentia_sensors/TerraEncoder.h>

namespace gazebo {

     class Joint;
     class Entity;

     class GazeboRosEncoder : public ModelPlugin {

          public:
               GazeboRosEncoder();
               ~GazeboRosEncoder();
               void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

          protected:
               virtual void UpdateChild();
               virtual void FiniChild();

          private:

               physics::WorldPtr world;
               physics::ModelPtr parent;
               event::ConnectionPtr update_connection_;

               physics::JointPtr joint_;
               std::string joint_name_;

               double wheel_diameter_;
               double ppr_;
               double c_ppr_;

               // ROS STUFF
               ros::NodeHandle* _nh;

               ros::Publisher enc_publisher_;
               terrasentia_sensors::TerraEncoder enc_;

               boost::mutex lock;

               std::string robot_namespace_;
               std::string enc_topic_;

               bool alive_;

               // Update Rate
               double update_rate_;
               double update_period_;
               common::Time last_update_time_;

     };

}


#endif /* GAZEBO_ROS_ENCODER_H_ */
