#ifndef _GAZEBO_ROS_REALSENSE_PLUGIN_
#define _GAZEBO_ROS_REALSENSE_PLUGIN_

#include <ros/ros.h>
#include <sdf/sdf.hh>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/rendering/DepthCamera.hh>

#include <string>
#include <memory>
#include <random>
#include <functional>
#include <boost/thread/mutex.hpp>
// #include <mutex>

#include <dynamic_reconfigure/server.h>
#include <gazebo_sensor_collection/SimulatedCameraD4XXConfig.h>

struct my_class{
     my_class() = default ; // uses the default member initialise
     my_class( std::size_t seed, double mean, double std_dev ) : rng(seed), distrib(mean,std_dev) {}

     double generate(){ return distrib(rng) ; }
     void change_param(double mean, double std_dev){
          distrib.param(decltype(distrib)::param_type{mean, std_dev});
     }
private:
     // default member initializer seeds the rng with a random value
     // 32 bits of randomness would be adequate for most programs
     std::mt19937 rng{ std::random_device{} () };
     // default constructor initialises distrib to a standard normal distribution (mean 0, stddev 1)
     std::normal_distribution<double> distrib;
};

namespace gazebo{

class GazeboRosRealsense : public ModelPlugin{
     private:
          boost::mutex _lock;
          physics::WorldPtr world;
          physics::ModelPtr _parent;
          ros::NodeHandle* rosnode_;

          double update_rate_;
          double depth_near_clip;
          double depth_far_clip;
          double depth_scale;

          bool use_noise_dist_squared_ = true;
          double depth_noise_mean_ = 0.0;
          double depth_noise_std_ = 0.01;
          double depth_noise_aux_factor_ = 1.0;

          // std::random_device rd{};
          // std::mt19937 generator_{rd()};
          // std::default_random_engine re_;
          // std::normal_distribution<double>* depth_noise_generator_;
          // std::function<double()>* noisyDepth_;
          my_class depth_noise_generator_;

          double baseline; // in mm
          double fx_color_;
          double fy_color_;
          double fx_depth_;
          double fy_depth_;

          std::string robot_namespace_;
          std::string camera_name_;
          std::string robot_base_frame_;

          std::string depth_frame_id_;
          std::string rgb_frame_id_;

          std::string depth_topic_;
          std::string rgb_topic_;
          std::string ir_topic_;
          std::string ir2_topic_;

          std::string depth_image_topic_;
          std::string rgb_image_topic_;
          std::string ir_image_topic_;
          std::string ir2_image_topic_;

          bool ir_streams_enabled_;

          std::string depth_name_;
          std::string rgb_name_;
          std::string ir_name_;
          std::string ir2_name_;

          image_transport::ImageTransport* itnode_;

          boost::shared_ptr<dynamic_reconfigure::Server<gazebo_sensor_collection::SimulatedCameraD4XXConfig> > _cfg_server;
          void cfgCallback(gazebo_sensor_collection::SimulatedCameraD4XXConfig &config, uint32_t level);
     public:
          GazeboRosRealsense();
          ~GazeboRosRealsense();

          virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
          virtual void OnNewDepthFrame();
          virtual void OnNewFrame(const rendering::CameraPtr cam, const transport::PublisherPtr pub);
          virtual void OnNewInfaredFrame(const rendering::CameraPtr cam, const transport::PublisherPtr pub);

          void OnUpdate();

          std::string extractCameraName(const std::string& name);
          sensor_msgs::CameraInfo cameraInfo(const sensor_msgs::Image& image, float horizontal_fov);
          sensor_msgs::CameraInfo cameraInfo(const sensor_msgs::Image& image, double focal_x, double focal_y);
     protected:

          boost::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
          image_transport::CameraPublisher color_pub_, ir1_pub_, ir2_pub_, depth_pub_;
          sensor_msgs::Image image_msg_, depth_msg_;

          rendering::CameraPtr colorCam;
          rendering::CameraPtr ired1Cam;
          rendering::CameraPtr ired2Cam;
          rendering::DepthCameraPtr depthCam;

          transport::NodePtr transportNode;
          std::vector<uint16_t> depthMap;
          transport::PublisherPtr depthPub;
          transport::PublisherPtr colorPub;
          transport::PublisherPtr ired1Pub;
          transport::PublisherPtr ired2Pub;
          event::ConnectionPtr newDepthFrameConn;
          event::ConnectionPtr newIred1FrameConn;
          event::ConnectionPtr newIred2FrameConn;
          event::ConnectionPtr newColorFrameConn;
          event::ConnectionPtr updateConnection;

}; // GazeboRosRealsense

} // namespace gazebo
#endif /* _GAZEBO_ROS_REALSENSE_PLUGIN_ */
