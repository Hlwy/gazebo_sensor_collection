#include "vision/gazebo_ros_realsense.h"

#include <sensor_msgs/fill_image.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/sensors/sensors.hh>



namespace gazebo{

// Register the plugin
GZ_REGISTER_MODEL_PLUGIN(GazeboRosRealsense)

GazeboRosRealsense::GazeboRosRealsense() : depth_noise_generator_(){
     this->depthCam = nullptr;
     this->ired1Cam = nullptr;
     this->ired2Cam = nullptr;
     this->colorCam = nullptr;
}

GazeboRosRealsense::~GazeboRosRealsense(){
     ROS_DEBUG_STREAM_NAMED("realsense_camera", "Unloaded");
     // delete this->depth_noise_generator_;
     _cfg_server.reset();
     delete rosnode_;
}

/*
██       ██████   █████  ██████
██      ██    ██ ██   ██ ██   ██
██      ██    ██ ███████ ██   ██
██      ██    ██ ██   ██ ██   ██
███████  ██████  ██   ██ ██████
*/

void GazeboRosRealsense::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
     std::cout << std::endl
          << "GazeboRosRealsense: The realsense_camera plugin is attach to model "
          << _model->GetName() << std::endl;

     /** --------------- Set Default Params ---------------------------------*/
     this->_parent = _model;
     this->world = _parent->GetWorld();

     this->robot_namespace_ = "/realsense";
     this->camera_name_ = "realsense";
     this->robot_base_frame_ = "base_footprint";

     {
          this->ir_streams_enabled_ = false;
          this->depth_topic_ = "depth";
          this->rgb_topic_ = "color";
          this->ir_topic_ = "infrared";
          this->ir2_topic_ = "infrared2";

          this->depth_image_topic_ = "image_rect_raw";
          this->rgb_image_topic_ = "image_raw";
          this->ir_image_topic_ = "image_raw";
          this->ir2_image_topic_ = "image_raw";

          this->depth_name_ = "depth";
          this->rgb_name_ = "color";
          this->ir_name_ = "ired1";
          this->ir2_name_ = "ired2";

          this->update_rate_ = 60.0;
          this->depth_near_clip = 0.1;
          this->depth_far_clip = 30.0;
          this->depth_scale = 0.001;
          this->baseline = 0.014732;

          this->fx_color_ = 596.391845703125;
          this->fy_color_ = 596.391845703125;

          this->fx_depth_ = 614.15625;
          this->fy_depth_ = 613.42626953125;

          this->depth_frame_id_ = "camera_aligned_optical_frame";
          this->rgb_frame_id_ = "camera_aligned_optical_frame";

          /** ------------------------ Get User-Defined Params --------------------*/
          if(!_sdf->HasElement("robotNamespace")){
               ROS_INFO_NAMED("gazebo_ros_realsense", "GazeboRosRealsense Plugin missing <robotNamespace>, defaults to \"%s\"",
               this->robot_namespace_.c_str());
          }else{ this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/"; }

          if(!_sdf->HasElement("robotBaseFrame")){
               ROS_WARN_NAMED("gazebo_ros_realsense", "GazeboRosRealsense Plugin (ns = %s) missing <robotBaseFrame>, defaults to \"%s\"",
               this->robot_namespace_.c_str(), this->robot_base_frame_.c_str());
          }else{ this->robot_base_frame_ = _sdf->GetElement("robotBaseFrame")->Get<std::string>(); }

          if(!_sdf->HasElement("cameraName")){
               ROS_WARN_NAMED("gazebo_ros_realsense", "GazeboRosRealsense Plugin (ns = %s) missing <cameraName>, defaults to \"%s\"",
               this->robot_namespace_.c_str(), this->camera_name_.c_str());
          }else{ this->camera_name_ = _sdf->GetElement("cameraName")->Get<std::string>(); }

          if(!_sdf->HasElement("depthName")){
               ROS_WARN_NAMED("gazebo_ros_realsense", "GazeboRosRealsense Plugin (ns = %s) missing <depthName>, defaults to \"%s\"",
               this->robot_namespace_.c_str(), this->depth_name_.c_str());
          }else{ this->depth_name_ = _sdf->GetElement("depthName")->Get<std::string>(); }

          if(!_sdf->HasElement("rgbName")){
               ROS_WARN_NAMED("gazebo_ros_realsense", "GazeboRosRealsense Plugin (ns = %s) missing <rgbName>, defaults to \"%s\"",
               this->robot_namespace_.c_str(), this->rgb_name_.c_str());
          }else{ this->rgb_name_ = _sdf->GetElement("rgbName")->Get<std::string>(); }

          if(!_sdf->HasElement("ir1Name")){
               ROS_WARN_NAMED("gazebo_ros_realsense", "GazeboRosRealsense Plugin (ns = %s) missing <ir1Name>, defaults to \"%s\"",
               this->robot_namespace_.c_str(), this->ir_name_.c_str());
          }else{ this->ir_name_ = _sdf->GetElement("ir1Name")->Get<std::string>(); }

          if(!_sdf->HasElement("ir2Name")){
               ROS_WARN_NAMED("gazebo_ros_realsense", "GazeboRosRealsense Plugin (ns = %s) missing <ir2Name>, defaults to \"%s\"",
               this->robot_namespace_.c_str(), this->ir2_name_.c_str());
          }else{ this->ir2_name_ = _sdf->GetElement("ir2Name")->Get<std::string>(); }

          if(!_sdf->HasElement("depthTopic")){
               ROS_WARN_NAMED("gazebo_ros_realsense", "GazeboRosRealsense Plugin (ns = %s) missing <depthTopic>, defaults to \"%s\"",
               this->robot_namespace_.c_str(), this->depth_topic_.c_str());
          }else{ this->depth_topic_ = _sdf->GetElement("depthTopic")->Get<std::string>(); }

          if(!_sdf->HasElement("rgbTopic")){
               ROS_WARN_NAMED("gazebo_ros_realsense", "GazeboRosRealsense Plugin (ns = %s) missing <rgbTopic>, defaults to \"%s\"",
               this->robot_namespace_.c_str(), this->rgb_topic_.c_str());
          }else{ this->rgb_topic_ = _sdf->GetElement("rgbTopic")->Get<std::string>(); }

          if(!_sdf->HasElement("ir1Topic")){
               ROS_WARN_NAMED("gazebo_ros_realsense", "GazeboRosRealsense Plugin (ns = %s) missing <ir1Topic>, defaults to \"%s\"",
               this->robot_namespace_.c_str(), this->ir_topic_.c_str());
          }else{ this->ir_topic_ = _sdf->GetElement("ir1Topic")->Get<std::string>(); }

          if(!_sdf->HasElement("ir2Topic")){
               ROS_WARN_NAMED("gazebo_ros_realsense", "GazeboRosRealsense Plugin (ns = %s) missing <ir2Topic>, defaults to \"%s\"",
               this->robot_namespace_.c_str(), this->ir2_topic_.c_str());
          }else{ this->ir2_topic_ = _sdf->GetElement("ir2Topic")->Get<std::string>(); }

          if(!_sdf->HasElement("depthImageTopic")){
               ROS_WARN_NAMED("gazebo_ros_realsense", "GazeboRosRealsense Plugin (ns = %s) missing <depthImageTopic>, defaults to \"%s\"",
               this->robot_namespace_.c_str(), this->depth_image_topic_.c_str());
          }else{ this->depth_image_topic_ = _sdf->GetElement("depthImageTopic")->Get<std::string>(); }

          if(!_sdf->HasElement("rgbImageTopic")){
               ROS_WARN_NAMED("gazebo_ros_realsense", "GazeboRosRealsense Plugin (ns = %s) missing <rgbImageTopic>, defaults to \"%s\"",
               this->robot_namespace_.c_str(), this->rgb_image_topic_.c_str());
          }else{ this->rgb_image_topic_ = _sdf->GetElement("rgbImageTopic")->Get<std::string>(); }

          if(!_sdf->HasElement("depthImageFrameId")){
               ROS_WARN_NAMED("gazebo_ros_realsense", "GazeboRosRealsense Plugin (ns = %s) missing <depthImageFrameId>, defaults to \"%s\"",
               this->robot_namespace_.c_str(), this->depth_frame_id_.c_str());
          }else{ this->depth_frame_id_ = _sdf->GetElement("depthImageFrameId")->Get<std::string>(); }

          if(!_sdf->HasElement("rgbImageFrameId")){
               ROS_WARN_NAMED("gazebo_ros_realsense", "GazeboRosRealsense Plugin (ns = %s) missing <rgbImageFrameId>, defaults to \"%s\"",
               this->robot_namespace_.c_str(), this->rgb_frame_id_.c_str());
          }else{ this->rgb_frame_id_ = _sdf->GetElement("rgbImageFrameId")->Get<std::string>(); }

          if(!_sdf->HasElement("ir1ImageTopic")){
               ROS_WARN_NAMED("gazebo_ros_realsense", "GazeboRosRealsense Plugin (ns = %s) missing <ir1ImageTopic>, defaults to \"%s\"",
               this->robot_namespace_.c_str(), this->ir_image_topic_.c_str());
          }else{ this->ir_image_topic_ = _sdf->GetElement("ir1ImageTopic")->Get<std::string>(); }

          if(!_sdf->HasElement("ir2ImageTopic")){
               ROS_WARN_NAMED("gazebo_ros_realsense", "GazeboRosRealsense Plugin (ns = %s) missing <ir2ImageTopic>, defaults to \"%s\"",
               this->robot_namespace_.c_str(), this->ir2_image_topic_.c_str());
          }else{ this->ir2_image_topic_ = _sdf->GetElement("ir2ImageTopic")->Get<std::string>(); }

          if(!_sdf->HasElement("updateRate")){
               ROS_WARN_NAMED("gazebo_ros_realsense", "GazeboRosRealsense Plugin (ns = %s) missing <updateRate>, defaults to %f",
               this->robot_namespace_.c_str(), this->update_rate_);
          }else{ this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>(); }

          if(!_sdf->HasElement("depthNearClip")){
               ROS_WARN_NAMED("gazebo_ros_realsense", "GazeboRosRealsense Plugin (ns = %s) missing <depthNearClip>, defaults to %f",
               this->robot_namespace_.c_str(), this->depth_near_clip);
          }else{ this->depth_near_clip = _sdf->GetElement("depthNearClip")->Get<double>(); }

          if(!_sdf->HasElement("depthFarClip")){
               ROS_WARN_NAMED("gazebo_ros_realsense", "GazeboRosRealsense Plugin (ns = %s) missing <depthFarClip>, defaults to %f",
               this->robot_namespace_.c_str(), this->depth_far_clip);
          }else{ this->depth_far_clip = _sdf->GetElement("depthFarClip")->Get<double>(); }

          if(!_sdf->HasElement("depthScale")){
               ROS_WARN_NAMED("gazebo_ros_realsense", "GazeboRosRealsense Plugin (ns = %s) missing <depthScale>, defaults to %f",
               this->robot_namespace_.c_str(), this->depth_scale);
          }else{ this->depth_scale = _sdf->GetElement("depthScale")->Get<double>(); }

          if(!_sdf->HasElement("baseline")){
               ROS_WARN_NAMED("gazebo_ros_realsense", "GazeboRosRealsense Plugin (ns = %s) missing <baseline>, defaults to %f mm",
               this->robot_namespace_.c_str(), this->baseline);
          }else{ this->baseline = _sdf->GetElement("baseline")->Get<double>(); }

          if(!_sdf->HasElement("focalLengthXColor")){
               ROS_WARN_NAMED("gazebo_ros_realsense", "GazeboRosRealsense Plugin (ns = %s) missing <focalLengthXColor>, defaults to %f",
               this->robot_namespace_.c_str(), this->fx_color_);
          }else{ this->fx_color_ = _sdf->GetElement("focalLengthXColor")->Get<double>(); }

          if(!_sdf->HasElement("focalLengthYColor")){
               ROS_WARN_NAMED("gazebo_ros_realsense", "GazeboRosRealsense Plugin (ns = %s) missing <focalLengthYColor>, defaults to %f",
               this->robot_namespace_.c_str(), this->fy_color_);
          }else{ this->fy_color_ = _sdf->GetElement("focalLengthYColor")->Get<double>(); }

          if(!_sdf->HasElement("focalLengthXDepth")){
               ROS_WARN_NAMED("gazebo_ros_realsense", "GazeboRosRealsense Plugin (ns = %s) missing <focalLengthXDepth>, defaults to %f",
               this->robot_namespace_.c_str(), this->fx_depth_);
          }else{ this->fx_depth_ = _sdf->GetElement("focalLengthXDepth")->Get<double>(); }

          if(!_sdf->HasElement("focalLengthYDepth")){
               ROS_WARN_NAMED("gazebo_ros_realsense", "GazeboRosRealsense Plugin (ns = %s) missing <focalLengthYDepth>, defaults to %f",
               this->robot_namespace_.c_str(), this->fy_depth_);
          }else{ this->fy_depth_ = _sdf->GetElement("focalLengthYDepth")->Get<double>(); }

          if(!_sdf->HasElement("depthNoiseMean")){
               ROS_WARN_NAMED("gazebo_ros_realsense", "GazeboRosRealsense Plugin (ns = %s) missing <depthNoiseMean>, defaults to %f",
               this->robot_namespace_.c_str(), this->depth_noise_mean_);
          }else{ this->depth_noise_mean_ = _sdf->GetElement("depthNoiseMean")->Get<double>(); }

          if(!_sdf->HasElement("depthNoiseStd")){
               ROS_WARN_NAMED("gazebo_ros_realsense", "GazeboRosRealsense Plugin (ns = %s) missing <depthNoiseStd>, defaults to %f",
               this->robot_namespace_.c_str(), this->depth_noise_std_);
          }else{ this->depth_noise_std_ = _sdf->GetElement("depthNoiseStd")->Get<double>(); }

          this->depth_noise_generator_.change_param(this->depth_noise_mean_, this->depth_noise_std_);
          // this->depth_noise_generator_  = new std::normal_distribution<double>(this->depth_noise_mean_, this->depth_noise_std_);
          // this->noisyDepth_ = new std::function<double()>(std::bind(this->depth_noise_generator_, this->re_));

          if(!_sdf->HasElement("enableStreamsIR")){
               ROS_WARN_NAMED("gazebo_ros_realsense", "GazeboRosRealsense Plugin (ns = %s) missing <enableStreamsIR>, defaults to %s",
               this->robot_namespace_.c_str(), this->ir_streams_enabled_ ? "true" : "false");
          }else{ this->ir_streams_enabled_ = _sdf->GetElement("enableStreamsIR")->Get<bool>(); }
     }
     /** ------------------------ Setup --------------------*/

     // Make sure the ROS node for Gazebo has already been initialized
     if(!ros::isInitialized()){
          ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_realsense.so' in the gazebo_sensor_collection package)");
          return;
     }
     ROS_INFO_NAMED("gazebo_ros_realsense", "Starting GazeboRosRealsense Plugin (ns = %s)", this->robot_namespace_.c_str());

     this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);
     sensors::SensorManager *smanager = sensors::SensorManager::Instance();

     // Get Cameras Renderers
     this->depthCam = std::dynamic_pointer_cast<sensors::DepthCameraSensor>(smanager->GetSensor(this->depth_name_))->DepthCamera();
     this->colorCam = std::dynamic_pointer_cast<sensors::CameraSensor>(smanager->GetSensor(this->rgb_name_))->Camera();
     // Check if camera renderers have been found successfuly
     if(!this->depthCam){
          std::cerr << "GazeboRosRealsense: Depth Camera has not been found" << std::endl;
          return;
     }
     if(!this->colorCam){
          std::cerr << "GazeboRosRealsense: Color Camera has not been found" << std::endl;
          return;
     }

     if(this->ir_streams_enabled_){
          this->ired1Cam = std::dynamic_pointer_cast<sensors::CameraSensor>(smanager->GetSensor(this->ir_name_))->Camera();
          this->ired2Cam = std::dynamic_pointer_cast<sensors::CameraSensor>(smanager->GetSensor(this->ir2_name_))->Camera();
          if(!this->ired1Cam){
               std::cerr << "GazeboRosRealsense: InfraRed Camera 1 has not been found" << std::endl;
               return;
          }
          if(!this->ired2Cam){
               std::cerr << "GazeboRosRealsense: InfraRed Camera 2 has not been found" << std::endl;
               return;
          }
     }

     // Resize Depth Map dimensions
     try{ this->depthMap.resize(this->depthCam->ImageWidth() * this->depthCam->ImageHeight());
     } catch(std::bad_alloc &e){
          std::cerr << "GazeboRosRealsense: depthMap allocation failed: " << e.what() << std::endl;
          return;
     }

     // Setup Transport Node
     this->transportNode = transport::NodePtr(new transport::Node());

     #if GAZEBO_MAJOR_VERSION < 9
     this->transportNode->Init(this->world->GetName());
     #else
     this->transportNode->Init(this->world->Name());
     #endif

     // Setup Publishers
     std::string rsTopicRoot = "~/" + this->_parent->GetName() + "/rs/stream/";

     this->depthPub = this->transportNode->Advertise<msgs::ImageStamped>(rsTopicRoot + this->depth_topic_, 1, this->update_rate_);
     this->colorPub = this->transportNode->Advertise<msgs::ImageStamped>(rsTopicRoot + this->rgb_topic_, 1, this->update_rate_);
     if(this->ir_streams_enabled_){
          this->ired1Pub = this->transportNode->Advertise<msgs::ImageStamped>(rsTopicRoot + this->ir_topic_, 1, this->update_rate_);
          this->ired2Pub = this->transportNode->Advertise<msgs::ImageStamped>(rsTopicRoot + this->ir2_topic_, 1, this->update_rate_);
     }

     // Listen to depth camera new frame event
     this->newDepthFrameConn = this->depthCam->ConnectNewDepthFrame(std::bind(&GazeboRosRealsense::OnNewDepthFrame, this));
     this->newColorFrameConn = this->colorCam->ConnectNewImageFrame(std::bind(&GazeboRosRealsense::OnNewFrame, this, this->colorCam, this->colorPub));
     if(this->ir_streams_enabled_){
          this->newIred1FrameConn = this->ired1Cam->ConnectNewImageFrame(std::bind(&GazeboRosRealsense::OnNewInfaredFrame, this, this->ired1Cam,this->ired1Pub));
          this->newIred2FrameConn = this->ired2Cam->ConnectNewImageFrame(std::bind(&GazeboRosRealsense::OnNewInfaredFrame, this, this->ired2Cam, this->ired2Pub));
     }
     // Listen to the update event
     this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosRealsense::OnUpdate, this));

     // initialize camera_info_manager
     this->camera_info_manager_.reset(new camera_info_manager::CameraInfoManager(*this->rosnode_, this->camera_name_));
     this->itnode_ = new image_transport::ImageTransport(*this->rosnode_);

     std::string depthtopic = this->camera_name_ + "/" + this->depth_name_ + "/" + this->depth_image_topic_;
     std::string rgbtopic = this->camera_name_ + "/" + this->rgb_name_ + "/" + this->rgb_image_topic_;
     std::string ir1topic = this->camera_name_ + "/" + this->ir_name_ + "/" + this->ir_image_topic_;
     std::string ir2topic = this->camera_name_ + "/" + this->ir2_name_ + "/" + this->ir2_image_topic_;

     this->depth_pub_ = this->itnode_->advertiseCamera(depthtopic, 2);
     this->color_pub_ = this->itnode_->advertiseCamera(rgbtopic, 2);
     if(this->ir_streams_enabled_){
          this->ir1_pub_ = this->itnode_->advertiseCamera(ir1topic, 2);
          this->ir2_pub_ = this->itnode_->advertiseCamera(ir2topic, 2);
     }

     _cfg_server.reset(new dynamic_reconfigure::Server<gazebo_sensor_collection::SimulatedCameraD4XXConfig>(ros::NodeHandle(*this->rosnode_, "gazebo_plugins/camera_d4xx")));
     _cfg_server->setCallback(boost::bind(&GazeboRosRealsense::cfgCallback, this, _1, _2));
     // this->_cfg_f = boost::bind(&GazeboRosRealsense::cfgCallback, this, _1, _2);
     // this->_cfg_server.setCallback(this->_cfg_f);
}

void GazeboRosRealsense::cfgCallback(gazebo_sensor_collection::SimulatedCameraD4XXConfig &config, uint32_t level){
     // std::lock_guard<std::mutex> lock(_lock);

     // if(config.verbose_update != this->_verbose_update){
     //      this->_verbose_update = config.verbose_update;
     // }
     // if(config.umap_sobelized_blur_size != this->vb->umapParams.sobel_blur_size){
     //      if(config.umap_sobelized_blur_size == 0) this->vb->umapParams.set_sobelize_blur_size(1);
     //      else this->vb->umapParams.set_sobelize_blur_size(config.umap_sobelized_blur_size);
     // }

     if(config.square_depth_noise != this->use_noise_dist_squared_){ this->use_noise_dist_squared_ = config.square_depth_noise; }

     if(config.depth_min != this->depth_near_clip){ this->depth_near_clip = config.depth_min; }
     if(config.depth_max != this->depth_far_clip){ this->depth_far_clip = config.depth_max; }

     if(config.depth_noise_mean != this->depth_noise_mean_){ this->depth_noise_mean_ = config.depth_noise_mean; }
     if(config.depth_noise_std != this->depth_noise_std_){ this->depth_noise_std_ = config.depth_noise_std; }
     this->depth_noise_generator_.change_param(this->depth_noise_mean_, this->depth_noise_std_);
     if(config.depth_noise_aux_factor != this->depth_noise_aux_factor_){ this->depth_noise_aux_factor_ = config.depth_noise_aux_factor; }
}

/*
 ██████  ███    ██ ███    ██ ███████ ██     ██ ███████ ██████   █████  ███    ███ ███████
██    ██ ████   ██ ████   ██ ██      ██     ██ ██      ██   ██ ██   ██ ████  ████ ██
██    ██ ██ ██  ██ ██ ██  ██ █████   ██  █  ██ █████   ██████  ███████ ██ ████ ██ █████
██    ██ ██  ██ ██ ██  ██ ██ ██      ██ ███ ██ ██      ██   ██ ██   ██ ██  ██  ██ ██
 ██████  ██   ████ ██   ████ ███████  ███ ███  ██      ██   ██ ██   ██ ██      ██ ███████
*/

void GazeboRosRealsense::OnNewFrame(const rendering::CameraPtr cam, const transport::PublisherPtr pub){
     common::Time current_time;
     #if GAZEBO_MAJOR_VERSION < 9
     current_time = this->world->GetSimTime();
     #else
     current_time = this->world->SimTime();
     #endif

     // identify camera
     std::string camera_id = extractCameraName(cam->Name());
     const std::map<std::string, image_transport::CameraPublisher*> camera_publishers = {
          {this->rgb_name_, &(this->color_pub_)},
     };
     const auto image_pub = camera_publishers.at(camera_id);

     // copy data into image
     // this->image_msg_.header.frame_id = "camera_" + camera_id + "_optical_frame";
     this->image_msg_.header.frame_id = this->rgb_frame_id_;
     this->image_msg_.header.stamp.sec = current_time.sec;
     this->image_msg_.header.stamp.nsec = current_time.nsec;

     // set image encoding
     const std::map<std::string, std::string> supported_image_encodings = {
          {"L_INT8", sensor_msgs::image_encodings::MONO8},
          {"RGB_INT8", sensor_msgs::image_encodings::RGB8},
     };
     const auto pixel_format = supported_image_encodings.at(cam->ImageFormat());

     // copy from simulation image to ROS msg
     fillImage(this->image_msg_, pixel_format, cam->ImageHeight(), cam->ImageWidth(),
          cam->ImageDepth() * cam->ImageWidth(),
          reinterpret_cast<const void*>(cam->ImageData())
     );

     // identify camera rendering
     const std::map<std::string, rendering::CameraPtr> cameras = {
          {this->rgb_name_, this->colorCam},
     };

     // publish to ROS
     // auto camera_info_msg = cameraInfo(this->image_msg_, cameras.at(camera_id)->HFOV().Radian());
     auto camera_info_msg = cameraInfo(this->image_msg_, this->fx_color_, this->fy_color_);
     image_pub->publish(this->image_msg_, camera_info_msg);
}
void GazeboRosRealsense::OnNewInfaredFrame(const rendering::CameraPtr cam, const transport::PublisherPtr pub){
     common::Time current_time;
     #if GAZEBO_MAJOR_VERSION < 9
     current_time = this->world->GetSimTime();
     #else
     current_time = this->world->SimTime();
     #endif

     // identify camera
     std::string camera_id = extractCameraName(cam->Name());
     const std::map<std::string, image_transport::CameraPublisher*> camera_publishers = {
          {this->ir_name_, &(this->ir1_pub_)},
          {this->ir2_name_, &(this->ir2_pub_)},
     };
     const auto image_pub = camera_publishers.at(camera_id);

     // copy data into image
     this->image_msg_.header.frame_id = "camera_" + camera_id + "_optical_frame";
     this->image_msg_.header.stamp.sec = current_time.sec;
     this->image_msg_.header.stamp.nsec = current_time.nsec;

     // set image encoding
     const std::map<std::string, std::string> supported_image_encodings = {
          {"L_INT8", sensor_msgs::image_encodings::MONO8},
          {"RGB_INT8", sensor_msgs::image_encodings::RGB8},
     };
     const auto pixel_format = supported_image_encodings.at(cam->ImageFormat());

     // copy from simulation image to ROS msg
     fillImage(this->image_msg_, pixel_format, cam->ImageHeight(), cam->ImageWidth(),
          cam->ImageDepth() * cam->ImageWidth(),
          reinterpret_cast<const void*>(cam->ImageData())
     );

     // identify camera rendering
     const std::map<std::string, rendering::CameraPtr> cameras = {
          {this->ir_name_, this->ired1Cam},
          {this->ir2_name_, this->ired2Cam},
     };

     // publish to ROS
     // auto camera_info_msg = cameraInfo(this->image_msg_, cameras.at(camera_id)->HFOV().Radian());
     auto camera_info_msg = cameraInfo(this->image_msg_, this->fx_color_, this->fy_color_);
     image_pub->publish(this->image_msg_, camera_info_msg);
}

/*
██████  ███████ ██████  ████████ ██   ██ ███████ ██████   █████  ███    ███ ███████
██   ██ ██      ██   ██    ██    ██   ██ ██      ██   ██ ██   ██ ████  ████ ██
██   ██ █████   ██████     ██    ███████ █████   ██████  ███████ ██ ████ ██ █████
██   ██ ██      ██         ██    ██   ██ ██      ██   ██ ██   ██ ██  ██  ██ ██
██████  ███████ ██         ██    ██   ██ ██      ██   ██ ██   ██ ██      ██ ███████
*/

void GazeboRosRealsense::OnNewDepthFrame(){
     // std::lock_guard<std::mutex> lock(_lock);

     common::Time current_time;
     // get current time
     #if GAZEBO_MAJOR_VERSION < 9
     current_time = this->world->GetSimTime();
     #else
     current_time = this->world->SimTime();
     #endif

     // Get Depth Map dimensions
     unsigned int imageSize = this->depthCam->ImageWidth() * this->depthCam->ImageHeight();
     // Instantiate message
     msgs::ImageStamped msg;

     // Convert Float depth data to RealSense depth data
     const float *depthDataFloat = this->depthCam->DepthData();
     for(unsigned int i = 0; i < imageSize; ++i){
          // Noise gain based on distance squared (i.e. the farther the depth -> the noiser the depth pixels should be )

          // double tmpNoiseGain = (double)( depthDataFloat[i] * depthDataFloat[i] );
          double tmpNoiseGain;
          double tmpDepthRatio = (double)( depthDataFloat[i] / this->depth_far_clip );
          if(this->use_noise_dist_squared_) tmpNoiseGain = (double)( tmpDepthRatio * tmpDepthRatio );
          else tmpNoiseGain = tmpDepthRatio;

          tmpNoiseGain = tmpNoiseGain * (double)(this->depth_noise_aux_factor_);
          double tmpNoiseVal = this->depth_noise_generator_.generate();
          // Check clipping and overflow
          if(depthDataFloat[i] < this->depth_near_clip || depthDataFloat[i] < 0){
               this->depthMap[i] = 0;
          } else if(depthDataFloat[i] > this->depth_far_clip || depthDataFloat[i] > this->depth_scale * UINT16_MAX){
               this->depthMap[i] = (uint16_t)(UINT16_MAX);
          } else{
               this->depthMap[i] = (uint16_t)( (depthDataFloat[i] + (tmpNoiseVal*tmpNoiseGain)) / this->depth_scale);
          }
     }

     // Pack realsense scaled depth map
     #if GAZEBO_MAJOR_VERSION < 9
     msgs::Set(msg.mutable_time(), this->world->GetSimTime());
     #else
     msgs::Set(msg.mutable_time(), this->world->SimTime());
     #endif
     msg.mutable_image()->set_width(this->depthCam->ImageWidth());
     msg.mutable_image()->set_height(this->depthCam->ImageHeight());
     msg.mutable_image()->set_pixel_format(common::Image::L_INT16);
     msg.mutable_image()->set_step(this->depthCam->ImageWidth() * this->depthCam->ImageDepth());
     msg.mutable_image()->set_data(this->depthMap.data(), sizeof(*this->depthMap.data()) * imageSize);

     // Publish realsense scaled depth map
     this->depthPub->Publish(msg);

     // copy data into image
     // this->depth_msg_.header.frame_id = "camera_" + this->depth_name_ + "_optical_frame";
     this->depth_msg_.header.frame_id = this->depth_frame_id_;
     this->depth_msg_.header.stamp.sec = current_time.sec;
     this->depth_msg_.header.stamp.nsec = current_time.nsec;

     // set image encoding
     std::string pixel_format = sensor_msgs::image_encodings::TYPE_16UC1;

     // copy from simulation image to ROS msg
     fillImage(this->depth_msg_, pixel_format,
          this->depthCam->ImageHeight(), this->depthCam->ImageWidth(),
          2 * this->depthCam->ImageWidth(),
          reinterpret_cast<const void*>(this->depthMap.data())
     );

     // publish to ROS
     sensor_msgs::CameraInfo depth_info_msg = cameraInfo(this->depth_msg_, this->fx_depth_, this->fy_depth_);
     depth_info_msg.P[3] = -1.0 * (this->fx_depth_ * this->baseline);
     depth_info_msg.D = {this->depth_scale, 0.0, 0.0, 0.0, 0.0};

     this->depth_pub_.publish(this->depth_msg_, depth_info_msg);
}

std::string GazeboRosRealsense::extractCameraName(const std::string& name){
     if(name.find(this->rgb_name_) != std::string::npos) return this->rgb_name_;
     if(name.find(this->ir_name_) != std::string::npos) return this->ir_name_;
     if(name.find(this->ir2_name_) != std::string::npos) return this->ir2_name_;

     ROS_ERROR("Unknown camera name");
     return this->rgb_name_;
}

sensor_msgs::CameraInfo GazeboRosRealsense::cameraInfo(const sensor_msgs::Image& image, float horizontal_fov){
     sensor_msgs::CameraInfo info_msg;

     info_msg.header = image.header;
     info_msg.height = image.height;
     info_msg.width = image.width;

     float focal = 0.5 * image.width / tan(0.5 * horizontal_fov);

     info_msg.K[0] = focal;
     info_msg.K[4] = focal;
     info_msg.K[2] = info_msg.width * 0.5;
     info_msg.K[5] = info_msg.height * 0.5;
     info_msg.K[8] = 1.;

     info_msg.P[0] = info_msg.K[0];
     info_msg.P[5] = info_msg.K[4];
     info_msg.P[2] = info_msg.K[2];
     info_msg.P[6] = info_msg.K[5];
     info_msg.P[10] = info_msg.K[8];

     return info_msg;
}
sensor_msgs::CameraInfo GazeboRosRealsense::cameraInfo(const sensor_msgs::Image& image, double focal_x, double focal_y){
     sensor_msgs::CameraInfo info_msg;

     info_msg.header = image.header;
     info_msg.height = image.height;
     info_msg.width = image.width;

     info_msg.K[0] = focal_x;
     info_msg.K[4] = focal_y;
     info_msg.K[2] = info_msg.width * 0.5;
     info_msg.K[5] = info_msg.height * 0.5;
     info_msg.K[8] = 1.0;

     info_msg.P[0] = info_msg.K[0];
     info_msg.P[5] = info_msg.K[4];
     info_msg.P[2] = info_msg.K[2];
     info_msg.P[6] = info_msg.K[5];
     info_msg.P[10] = info_msg.K[8];
     return info_msg;
}

void GazeboRosRealsense::OnUpdate(){}

} // namespace gazebo
