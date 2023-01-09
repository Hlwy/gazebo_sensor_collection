#ifndef CUSTOM_GAZEBO_PLUGINS_GAZEBO_ROS_IMU_MOD_HEC_H
#define CUSTOM_GAZEBO_PLUGINS_GAZEBO_ROS_IMU_MOD_HEC_H

// NOTE: Porting of CBQ functionality to ROS 2 is still pending.
// #define USE_CBQ
// #ifdef USE_CBQ
// #include <ros/callback_queue.h>
// #include <ros/advertise_options.h>
// #endif

#include <gazebo/common/Plugin.hh>
#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/empty.hpp>
#include <gazebo_sensor_collection/srv/set_bias.hpp>
#include <gazebo_sensor_collection/sensor_model.h>
#include <gazebo_sensor_collection/update_timer.h>

namespace gazebo{

class GazeboRosIMUModHec : public ModelPlugin{
public:
    GazeboRosIMUModHec();
    virtual ~GazeboRosIMUModHec();

protected:
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void Reset();
    virtual void Update();

    rcl_interfaces::msg::SetParametersResult parametersChangedCallback(const std::vector<rclcpp::Parameter> & parameters);

private:
    boost::mutex lock; /// \brief A mutex to lock access to fields that are used in message callbacks

    std::string namespace_;   /// \brief for setting ROS name space
    physics::WorldPtr world;  /// \brief The parent World
    physics::LinkPtr link;    /// \brief The link referred to by this plugin

    std::string link_name_;   /// \brief store link name
    std::string frame_id_;    /// \brief frame id
    /// \brief topic name
    std::string topic_;
    std::string bias_topic_;

    /// \brief pointer to ros node
    gazebo_ros::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr bias_pub_;
    /// \brief ros message
    sensor_msgs::msg::Imu imuMsg;
    sensor_msgs::msg::Imu biasMsg;


    /// \brief Sensor models
    SensorModel3 accelModel;
    SensorModel3 rateModel;
    SensorModel yawModel;
    double GaussianKernel(double mu,double sigma);    /// \brief Gaussian noise generator

    /// \brief allow specifying constant xyz and rpy offsets
#if (GAZEBO_MAJOR_VERSION >= 8)
    ignition::math::Pose3d offset_;
#else
    math::Pose offset_;
#endif

    /// \brief save current body/physics state
#if (GAZEBO_MAJOR_VERSION >= 8)
    ignition::math::Quaterniond orientation;
    ignition::math::Vector3d velocity;
    ignition::math::Vector3d accel;
    ignition::math::Vector3d rate;
    ignition::math::Vector3d gravity;
#else
    math::Quaternion orientation;
    math::Vector3 velocity;
    math::Vector3 accel;
    math::Vector3 rate;
    math::Vector3 gravity;
#endif

    /// \brief call back when using service
    bool ServiceCallback(
        const std::shared_ptr<std_srvs::srv::Empty::Request> req,
        std::shared_ptr<std_srvs::srv::Empty::Response> res
    );

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_;
    std::string serviceName;
    /// \brief Bias service callbacks
    bool SetAccelBiasCallback(
        const std::shared_ptr<gazebo_sensor_collection::srv::SetBias::Request> req,
        std::shared_ptr<gazebo_sensor_collection::srv::SetBias::Response> res
    );
    bool SetRateBiasCallback(
        const std::shared_ptr<gazebo_sensor_collection::srv::SetBias::Request> req,
        std::shared_ptr<gazebo_sensor_collection::srv::SetBias::Response> res
    );
    rclcpp::Service<gazebo_sensor_collection::srv::SetBias>::SharedPtr accelBiasService;
    rclcpp::Service<gazebo_sensor_collection::srv::SetBias>::SharedPtr rateBiasService;

    // / \note Porting of CBQ functionality to ROS 2 is still pending.
    // #ifdef USE_CBQ
    // ros::CallbackQueue callback_queue_;
    // void CallbackQueueThread();
    // boost::thread callback_queue_thread_;
    // #endif
    UpdateTimer updateTimer;
    event::ConnectionPtr updateConnection;

    /// \brief Parameter changes callback
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
};

}

#endif // CUSTOM_GAZEBO_PLUGINS_GAZEBO_ROS_IMU_MOD_HEC_H
