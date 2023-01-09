#ifndef GAZEBO_SENSOR_COLLECTION_SKID_STEER_DRIVE_W_ENCODER_H_
#define GAZEBO_SENSOR_COLLECTION_SKID_STEER_DRIVE_W_ENCODER_H_

// #include <map>
//
// #include <gazebo/common/common.hh>
// #include <gazebo/physics/physics.hh>
//
// // ROS
// #include <ros/ros.h>
// #include <tf/transform_broadcaster.h>
// #include <tf/transform_listener.h>
// #include <geometry_msgs/Twist.h>
// #include <nav_msgs/Odometry.h>
// #include <nav_msgs/OccupancyGrid.h>
//
// // Custom Callback Queue
// #include <ros/callback_queue.h>
// #include <ros/advertise_options.h>
//
// // Boost
// #include <boost/thread.hpp>
// #include <boost/bind.hpp>

#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sdf/sdf.hh>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <gazebo_sensor_collection/EncoderData.h>

#include <memory>
#include <string>
#include <vector>

namespace gazebo {

class GazeboRosDiffDrivePrivate{
public:
    /// Indicates where the odometry info is coming from
    enum OdomSource{
        ENCODER = 0,
        WORLD = 1,
    };
    /// Indicates which wheel
    enum {
        RIGHT_FRONT=0,
        LEFT_FRONT=1,
        RIGHT_REAR=2,
        LEFT_REAR=3,
    };

    /// Callback to be called at every simulation iteration.
    /// \param[in] _info Updated simulation info.
    void OnUpdate(const gazebo::common::UpdateInfo & _info);
    /// Callback when a velocity command is received.
    /// \param[in] _msg Twist command message.
    void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg);
    /// Update odometry according to encoder.
    /// \param[in] _current_time Current simulation time
    void UpdateOdometryEncoder(const gazebo::common::Time & _current_time);
    /// Update wheel velocities according to latest target velocities.
    void UpdateWheelVelocities();
    /// Update odometry according to world
    void UpdateOdometryWorld();
    /// Publish odometry transforms \param[in] _current_time Current simulation time
    void PublishOdometryTf(const gazebo::common::Time & _current_time);
    /// Publish trasforms for the wheels \param[in] _current_time Current simulation time
    void PublishWheelsTf(const gazebo::common::Time & _current_time);
    /// Publish odometry messages \param[in] _current_time Current simulation time
    void PublishOdometryMsg(const gazebo::common::Time & _current_time);







      gazebo_ros::Node::SharedPtr ros_node_; /// A pointer to the GazeboROS node.
      rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_; /// Subscriber to command velocities
      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_; /// Odometry publisher
      gazebo::event::ConnectionPtr update_connection_; /// Connection to event called at every world iteration.

      std::vector<double> wheel_separation_; /// Distance between the wheels, in meters.
      std::vector<double> wheel_diameter_; /// Diameter of wheels, in meters.
      double max_wheel_torque_; /// Maximum wheel torque, in Nm.
      double max_wheel_accel_; /// Maximum wheel acceleration
      std::vector<double> desired_wheel_speed_; /// Desired wheel speed.
      std::vector<double> wheel_speed_instr_; /// Speed sent to wheel.
      std::vector<gazebo::physics::JointPtr> joints_; /// Pointers to wheel joints.
      gazebo::physics::ModelPtr model_; /// Pointer to model.
      std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_; /// To broadcast TFs
      std::mutex lock_; /// Protect variables accessed on callbacks.
      double target_x_{0.0}; /// Linear velocity in X received on command (m/s).
      double target_rot_{0.0}; /// Angular velocity in Z received on command (rad/s).
      double update_period_; /// Update period in seconds.
      gazebo::common::Time last_update_time_; /// Last update time.
      geometry_msgs::msg::Pose2D pose_encoder_; /// Keep encoder data.
      std::string odometry_frame_; /// Odometry frame ID
      gazebo::common::Time last_encoder_update_; /// Last time the encoder was updated
      OdomSource odom_source_; /// Either ENCODER or WORLD
      nav_msgs::msg::Odometry odom_; /// Keep latest odometry message
      std::string robot_base_frame_; /// Robot base frame ID
      bool publish_odom_; /// True to publish odometry messages.
      bool publish_wheel_tf_; /// True to publish wheel-to-base transforms.
      bool publish_odom_tf_; /// True to publish odom-to-world transforms.
      unsigned int num_wheel_pairs_; /// Store number of wheel pairs
      double covariance_[3]; /// Covariance in odometry
    };

class GazeboSkidSteerDriveWEncoder : public ModelPlugin {

public:
    GazeboSkidSteerDriveWEncoder();
    ~GazeboSkidSteerDriveWEncoder();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

protected:

    virtual void UpdateChild();
    virtual void FiniChild();

private:
    void publishOdometry(double step_time);
    void getWheelVelocities();

    physics::WorldPtr world;
    physics::ModelPtr parent;
    event::ConnectionPtr update_connection_;

    std::string left_front_joint_name_;
    std::string right_front_joint_name_;
    std::string left_rear_joint_name_;
    std::string right_rear_joint_name_;

    double wheel_separation_;
    double wheel_diameter_;
    double torque;
    double wheel_speed_[4];
    double ppr_;
    double c_ppr_;

    physics::JointPtr joints[4];

    // ROS STUFF
    ros::NodeHandle* rosnode_;

    ros::Publisher odometry_publisher_;

    ros::Publisher enc_fr_publisher_;
    ros::Publisher enc_fl_publisher_;
    ros::Publisher enc_rr_publisher_;
    ros::Publisher enc_rl_publisher_;

    ros::Subscriber cmd_vel_subscriber_;
    tf::TransformBroadcaster *transform_broadcaster_;
    nav_msgs::Odometry odom_;
    gazebo_sensor_collection::EncoderData enc_fr;
    gazebo_sensor_collection::EncoderData enc_fl;
    gazebo_sensor_collection::EncoderData enc_rr;
    gazebo_sensor_collection::EncoderData enc_rl;
    std::string tf_prefix_;
    bool broadcast_tf_;

    boost::mutex lock;

    std::string robot_namespace_;
    std::string command_topic_;
    std::string odometry_topic_;
    std::string odometry_frame_;
    std::string robot_base_frame_;

    std::string fr_topic;
    std::string fl_topic;
    std::string rr_topic;
    std::string rl_topic;

    // Custom Callback Queue
    ros::CallbackQueue queue_;
    boost::thread callback_queue_thread_;
    void QueueThread();

    // DiffDrive stuff
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

    double x_;
    double rot_;
    bool alive_;

    // Update Rate
    double update_rate_;
    double update_period_;
    common::Time last_update_time_;

    double covariance_x_;
    double covariance_y_;
    double covariance_yaw_;
};
}

#endif /* GAZEBO_SENSOR_COLLECTION_SKID_STEER_DRIVE_W_ENCODER_H_ */
