#ifndef _GAZEBO_ROS_REALSENSE_PLUGIN_
#define _GAZEBO_ROS_REALSENSE_PLUGIN_

#include <sdf/sdf.hh>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/sensors/sensors.hh>

#include <string>
#include <memory>

namespace gazebo{

class GazeboRosRealsense : public ModelPlugin{
     private:
          physics::WorldPtr world;
          physics::ModelPtr _parent;
          ros::NodeHandle* rosnode_;

          double update_rate_;
          double depth_near_clip;
          double depth_far_clip;
          double depth_scale;
          double baseline; // in mm

          std::string robot_namespace_;
          std::string camera_name_;
          std::string robot_base_frame_;

          std::string depth_topic_;
          std::string rgb_topic_;
          std::string ir_topic_;
          std::string ir2_topic_;

          std::string depth_name_;
          std::string rgb_name_;
          std::string ir_name_;
          std::string ir2_name_;

          image_transport::ImageTransport* itnode_;

     public:
          GazeboRosRealsense();
          ~GazeboRosRealsense();

          virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
          virtual void OnNewDepthFrame();
          virtual void OnNewFrame(const rendering::CameraPtr cam, const transport::PublisherPtr pub);

          void OnUpdate();

          std::string extractCameraName(const std::string& name);
          sensor_msgs::CameraInfo cameraInfo(const sensor_msgs::Image& image, float horizontal_fov);
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
