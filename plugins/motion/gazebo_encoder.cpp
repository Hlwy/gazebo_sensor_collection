#include <algorithm>
#include <assert.h>

#include "../include/motion/gazebo_encoder.h"

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

namespace gazebo {

     GazeboRosEncoder::GazeboRosEncoder(){}

     // Destructor
     GazeboRosEncoder::~GazeboRosEncoder(){
          delete _nh;
     }

  // Load the controller
     void GazeboRosEncoder::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){

          this->parent = _parent;
          this->world = _parent->GetWorld();

          this->robot_namespace_ = "";
          this->joint_name_ = "joint";
          this->enc_topic_ = "encoder";
          this->wheel_diameter_ = 0.15;
          this->update_rate_ = 100.0;
          this->ppr_ = 150;


          /** =====================================
          *           Parse SDF/URDF Tags
          * ======================================= */
          if(!_sdf->HasElement("robotNamespace")){
               ROS_INFO_NAMED("sim_encoder", "GazeboRosEncoder Plugin missing <robotNamespace>, defaults to \"%s\"",
                    this->robot_namespace_.c_str());
          } else{
               this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
          }

          // TODO write error if joint doesn't exist!
          if(!_sdf->HasElement("joint")){
               ROS_WARN_NAMED("sim_encoder", "GazeboRosEncoder Plugin (ns = %s) missing <leftFrontJoint>, defaults to \"%s\"",
                    this->robot_namespace_.c_str(), this->joint_name_.c_str());
          } else{
               this->joint_name_ = _sdf->GetElement("joint")->Get<std::string>();
          }

          // TODO get this from robot_description
          if(!_sdf->HasElement("wheelDiameter")){
               ROS_WARN_NAMED("sim_encoder", "GazeboRosEncoder Plugin (ns = %s) missing <wheelDiameter>, defaults to %f",
                    this->robot_namespace_.c_str(), this->wheel_diameter_);
          } else{
               this->wheel_diameter_ = _sdf->GetElement("wheelDiameter")->Get<double>();
          }

          if(!_sdf->HasElement("pulsesPR")){
               ROS_WARN_NAMED("sim_encoder", "GazeboRosEncoder Plugin (ns = %s) missing <pulsesPR>, defaults to %f",
                    this->robot_namespace_.c_str(), this->ppr_);
          } else{
               this->ppr_ = _sdf->GetElement("pulsesPR")->Get<double>();
          }

          if(!_sdf->HasElement("encoderTopic")){
               ROS_WARN_NAMED("sim_encoder", "GazeboRosEncoder Plugin (ns = %s) missing <odometryTopic>, defaults to \"%s\"",
                    this->robot_namespace_.c_str(), this->enc_topic_.c_str());
          } else{
               this->enc_topic_ = _sdf->GetElement("encoderTopic")->Get<std::string>();
          }

          if(!_sdf->HasElement("updateRate")){
               ROS_WARN_NAMED("sim_encoder", "GazeboRosEncoder Plugin (ns = %s) missing <updateRate>, defaults to %f",
                    this->robot_namespace_.c_str(), this->update_rate_);
          } else{
               this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();
          }

          // Initialize update rate stuff
          if(this->update_rate_ > 0.0){
               this->update_period_ = 1.0 / this->update_rate_;
          } else{
               this->update_period_ = 0.0;
          }

          this->joint_ = this->parent->GetJoint(joint_name_);
          if(!this->joint_){
               char error[200];
               snprintf(error, 200,
                    "GazeboRosEncoder Plugin (ns = %s) couldn't get joint named \"%s\"",
                    this->robot_namespace_.c_str(), this->joint_name_.c_str());
               gzthrow(error);
          }

          // Initialize Encoders
          enc_.position = 0;
          enc_.speed = 0;
          enc_.qpps = 0;

          alive_ = true;
          last_update_time_ = this->world->GetSimTime();
          c_ppr_ = ppr_ / (2*M_PI); // Conversion Factor

          // Make sure the ROS node for Gazebo has already been initialized
          if(!ros::isInitialized()){
               ROS_FATAL_STREAM_NAMED("sim_encoder", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
                    << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
               return;
          }

          _nh = new ros::NodeHandle(this->robot_namespace_);
          ROS_INFO_NAMED("sim_encoder", "Starting GazeboRosEncoder Plugin (ns = %s)", this->robot_namespace_.c_str());

          enc_publisher_ = _nh->advertise<gazebo_sensor_collection::EncoderData>(enc_topic_, 1);

          // listen to the update event (broadcast every simulation iteration)
          this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosEncoder::UpdateChild, this));

     }

     // Update the controller
     void GazeboRosEncoder::UpdateChild(){
          double posi;
          double angul;
          double radius = this->wheel_diameter_ / 2.0;

          common::Time current_time = this->world->GetSimTime();
          double seconds_since_last_update = (current_time - last_update_time_).Double();

          if(seconds_since_last_update > update_period_){
               ros::Time current_ros_time = ros::Time::now();

               posi = joint_->GetAngle(0).Radian();
               angul = joint_->GetVelocity(0);

               enc_.stamp = current_ros_time;
               enc_.id = 0;
               enc_.position = posi;
               enc_.speed = angul*radius;
               enc_.qpps = angul * c_ppr_;

               // ROS_INFO_NAMED("sim_encoder", "Wheel Speeds [rad/s]: %.4f, %.4f, %.4f, %.4f", angul[0],angul[1],angul[2],angul[3]);
               // ROS_INFO_NAMED("sim_encoder", "Wheel Positions [radians]: %.4f, %.4f, %.4f, %.4f", posi[0],posi[1],posi[2],posi[3]);

               enc_publisher_.publish(enc_);
               last_update_time_+= common::Time(update_period_);

          }
     }

     // Finalize the controller
     void GazeboRosEncoder::FiniChild(){
          alive_ = false;
          _nh->shutdown();
     }

     GZ_REGISTER_MODEL_PLUGIN(GazeboRosEncoder)
}
