//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include "../include/gps/terra_ros_gps_with_dropout.h"

// WGS84 constants
static const double equatorial_radius = 6378137.0;
static const double flattening = 1.0/298.257223563;
static const double excentrity2 = 2*flattening - flattening*flattening;

// default reference position
static const double DEFAULT_REFERENCE_LATITUDE  = 49.9;
static const double DEFAULT_REFERENCE_LONGITUDE = 8.9;
static const double DEFAULT_REFERENCE_HEADING   = 0.0;
static const double DEFAULT_REFERENCE_ALTITUDE  = 0.0;

static std::ostringstream stream;
static int ros_quick_info_count = 0;
#define ROS_QUICK_INFO(x) stream << ros_quick_info_count++ << ". " << x << std::endl; ROS_INFO(stream.str().c_str())

namespace gazebo {

bool PeriodicNoiseBox::contains(float x, float y, float z){
	POINT p;

	if(this->top() > z || this->bottom() < z){
		return false;
	}

	p.x = x;
	p.y = y;

	POINT rot = rotate_point(this->center.cartesian.x, this->center.cartesian.y, -this->rotation.xy_rotation, p);

	//ROS_QUICK_INFO("rx, ry ... " << rot.x << ", " << rot.y);

	if(rot.x > this->center.cartesian.x + this->dimensions.width / 2){
		return false;
	}
	if(rot.x < this->center.cartesian.x - this->dimensions.width / 2){
		return false;
	}
	if(rot.y > this->center.cartesian.y + this->dimensions.length / 2){
		return false;
	}
	if(rot.y < this->center.cartesian.y - this->dimensions.length / 2){
		return false;
	}

	return true;
};

GazeboRosGpsWithGeofencing::GazeboRosGpsWithGeofencing()
{
	this->viz_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("viz_gps_pts", 1, true);
	this->measurementCount = 0;

	this->marker_.header.frame_id = "odom";
	this->marker_.header.stamp = ros::Time();
	this->marker_.ns = "gps_markers";
	this->marker_.type = visualization_msgs::Marker::ARROW;
	this->marker_.action = visualization_msgs::Marker::ADD;
	this->marker_.pose.orientation.x = 0;
	this->marker_.pose.orientation.y = 0;
	this->marker_.pose.orientation.z = 0;
	this->marker_.pose.orientation.w = 0;
	this->marker_.scale.x = 0.1;
	this->marker_.scale.y = 0.1;
	this->marker_.scale.z = 0.1;

	this->pos_marker_.header.frame_id = "odom";
	this->pos_marker_.header.stamp = ros::Time();
	this->pos_marker_.ns = "pos_markers";
	this->pos_marker_.type = visualization_msgs::Marker::CYLINDER;
	this->pos_marker_.action = visualization_msgs::Marker::ADD;
	this->pos_marker_.pose.orientation.x = 0;
	this->pos_marker_.pose.orientation.y = 0;
	this->pos_marker_.pose.orientation.z = 0;
	this->pos_marker_.pose.orientation.w = 0;
	this->pos_marker_.scale.x = 0.1;
	this->pos_marker_.scale.y = 0.1;
	this->pos_marker_.scale.z = 0.1;


}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosGpsWithGeofencing::~GazeboRosGpsWithGeofencing()
{
	updateTimer.Disconnect(updateConnection);

	dynamic_reconfigure_server_position_.reset();
	dynamic_reconfigure_server_velocity_.reset();
	dynamic_reconfigure_server_status_.reset();

	node_handle_->shutdown();
	delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosGpsWithGeofencing::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
	int i = 0;
	int j = 0;
	PeriodicNoiseBox* box = nullptr;
	sdf::ElementPtr p = nullptr;
	bool success = false;

	BiomassDensityGPSNoiseCoefficient* density_char;
	BiomassHeightGPSNoiseCoefficient* height_char;
	world = _model->GetWorld();

	// load parameters
	if (!_sdf->HasElement("robotNamespace"))
		namespace_.clear();
	else
		namespace_ = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString();

	ROS_QUICK_INFO("Configuring GPS noise and dropout behavior...");

	//	while(p = _sdf->GetNextElement() && i < 100){
	//		ROS_QUICK_INFO("" << p->GetName());
	//		i++;
	//	}

	if(_sdf->HasElement("field_gps_noise_characteristics")){
		for(i=0;i<99;i++){
			p = _sdf->GetNextElement("biomass_density_noise_coefficient");
			if(!p){
				break;
			}
			density_char = new BiomassDensityGPSNoiseCoefficient;
			success = p->GetElement("r1")->GetValue()->Get(density_char->density_range.min);
			success = p->GetElement("r2")->GetValue()->Get(density_char->density_range.max);
			success = p->GetElement("mean")->GetValue()->Get(density_char->mean);
			success = p->GetElement("variance")->GetValue()->Get(density_char->variance);
			this->field_noise_characteristics.biomass_density_gps_noise_coefficients.push_back(density_char);
		}
		for(i=0;i<99;i++){
			p = _sdf->GetNextElement("biomass_height_noise_coefficient");
			if(!p){
				break;
			}
			height_char = new BiomassHeightGPSNoiseCoefficient;
			success = p->GetElement("r1")->GetValue()->Get(height_char->height_range.min);
			success = p->GetElement("r2")->GetValue()->Get(height_char->height_range.max);
			success = p->GetElement("mean")->GetValue()->Get(height_char->mean);
			success = p->GetElement("variance")->GetValue()->Get(height_char->variance);
			this->field_noise_characteristics.biomass_height_gps_noise_coefficients.push_back(height_char);

			ROS_QUICK_INFO("Height coefficient: " << i << " \n Range, mean, variance ... [" << height_char->height_range.min << ", " \
					<< height_char->height_range.max << "] , " << height_char->mean << ", " << height_char->variance);
		}
	}
	else{
		ROS_QUICK_INFO("No GPS noise characteristics found...");
	}

	ROS_QUICK_INFO("Loading periodic noise box objects...");

	for(i=0;i<99;i++){
		p = _sdf->GetNextElement("periodic_noise_box");
		if(!p){
			break;
		}
		box = new PeriodicNoiseBox;

		success = p->GetElement("cx")->GetValue()->Get(box->center.cartesian.x);
		success = p->GetElement("cy")->GetValue()->Get(box->center.cartesian.y);
		success = p->GetElement("cz")->GetValue()->Get(box->center.cartesian.z);
		success = p->GetElement("w")->GetValue()->Get(box->dimensions.width);
		success = p->GetElement("l")->GetValue()->Get(box->dimensions.length);
		success = p->GetElement("h")->GetValue()->Get(box->dimensions.height);
		success = p->GetElement("rot")->GetValue()->Get(box->rotation.xy_rotation);
		success = p->GetElement("nps")->GetValue()->Get(box->static_parameters.noise_period_s );
		success = p->GetElement("nmpm")->GetValue()->Get(box->static_parameters.noise_mean_periodic_mean);
		success = p->GetElement("nmpv")->GetValue()->Get(box->static_parameters.noise_mean_periodic_variance);
		success = p->GetElement("nvpm")->GetValue()->Get(box->static_parameters.noise_variance_periodic_mean);
		success = p->GetElement("nvpv")->GetValue()->Get(box->static_parameters.noise_variance_perioidic_variance);

		this->noiseBoxes.push_back(box);
	}

	float m = 0;
	float v = 0;

	ROS_QUICK_INFO("Loading field plot noise box objects...");

	for(i=0;i<99;i++){
		double temp1, temp2;
		p = _sdf->GetNextElement("plot");
		if(!p){
			break;
		}
		box = new PeriodicNoiseBox;

		success = p->GetElement("x")->GetValue()->Get(box->center.cartesian.x);
		success = p->GetElement("y")->GetValue()->Get(box->center.cartesian.y);
		success = p->GetElement("z")->GetValue()->Get(box->center.cartesian.z);
		success = p->GetElement("row_width")->GetValue()->Get(temp1);
		success = p->GetElement("row_count")->GetValue()->Get(temp2);
		box->dimensions.width = temp1 * (temp2-1);
		success = p->GetElement("row_length")->GetValue()->Get(box->dimensions.length);
		success = p->GetElement("average_height")->GetValue()->Get(box->dimensions.height);
		success = p->GetElement("rotation")->GetValue()->Get(box->rotation.xy_rotation);
		box->static_parameters.noise_period_s = -1;
		box->static_parameters.noise_mean_periodic_mean = -1;
		box->static_parameters.noise_mean_periodic_variance = -1;
		box->static_parameters.noise_variance_periodic_mean = -1;
		box->static_parameters.noise_variance_perioidic_variance = -1;

		m = 0;
		v = 0;
		for(j=0;j<this->field_noise_characteristics.biomass_height_gps_noise_coefficients.size();j++){
			height_char = this->field_noise_characteristics.biomass_height_gps_noise_coefficients[j];
			if(box->dimensions.height >= height_char->height_range.min && box->dimensions.height <= height_char->height_range.max){
				m = height_char->mean;
				v = height_char->variance;
			}
		}

		box->dynamic_parameters.noise_mean = m;
		box->dynamic_parameters.noise_variance = v;

		this->noiseBoxes.push_back(box);

		ROS_QUICK_INFO("GPS noise box for plot: " << i \
				<< " \n [x, y, z], [width, length, height], rotation, [noise-mean, noise-variance] ...\n" \
				<< "[ " << box->center.cartesian.x << "," << box->center.cartesian.y << "," << box->center.cartesian.z << "]\n" \
				<< "[ " << box->dimensions.width << "," << box->dimensions.length << "," << box->dimensions.height << "]\n" \
				<< box->rotation.xy_rotation << "\n" \
				<< "[ " << box->dynamic_parameters.noise_mean << "," << box->dynamic_parameters.noise_variance << "]\n");
	}

	//	height_char = new BiomassHeightGPSNoiseCoefficient;
	//	height_char->height_range.min = 0;	height_char->height_range.max = .2;	height_char->mean = 0;	height_char->variance = 1;
	//	this->field_noise_characteristics.biomass_height_gps_noise_coefficients.push_back(height_char);
	//	height_char = new BiomassHeightGPSNoiseCoefficient;
	//	height_char->height_range.min = .2;	height_char->height_range.max = .5;	height_char->mean = 0;	height_char->variance = 1.2;
	//	this->field_noise_characteristics.biomass_height_gps_noise_coefficients.push_back(height_char);
	//	height_char = new BiomassHeightGPSNoiseCoefficient;
	//	height_char->height_range.min = .5;	height_char->height_range.max = 1;	height_char->mean = 0;	height_char->variance = 1.5;
	//	this->field_noise_characteristics.biomass_height_gps_noise_coefficients.push_back(height_char);
	//	height_char = new BiomassHeightGPSNoiseCoefficient;
	//	height_char->height_range.min = 1;	height_char->height_range.max = 2;	height_char->mean = 0;	height_char->variance = 4;
	//	this->field_noise_characteristics.biomass_height_gps_noise_coefficients.push_back(height_char);
	//	height_char = new BiomassHeightGPSNoiseCoefficient;
	//	height_char->height_range.min = 2;	height_char->height_range.max = -1;	height_char->mean = 0;	height_char->variance = 5;
	//	this->field_noise_characteristics.biomass_height_gps_noise_coefficients.push_back(height_char);

	float pw = .762*3;

	box = new PeriodicNoiseBox;	box->center.cartesian.x = pw / 2;	box->center.cartesian.y = 5;	box->center.cartesian.z = -1.5;
	box->dimensions.height = 7;	box->dimensions.length = 10;	box->dimensions.width = pw;	box->dynamic_parameters.noise_mean = 0;
	box->dynamic_parameters.noise_variance = 4;	box->static_parameters.noise_period_s = -1; box->rotation.xy_rotation = 0;
	this->noiseBoxes.push_back(box);
	box = new PeriodicNoiseBox;	box->center.cartesian.x = pw / 2 + 2 + pw;	box->center.cartesian.y = 5;	box->center.cartesian.z = -1.5;
	box->dimensions.height = 7;	box->dimensions.length = 10;	box->dimensions.width = pw;	box->dynamic_parameters.noise_mean = 0;
	box->dynamic_parameters.noise_variance = 4;	box->static_parameters.noise_period_s = -1;box->rotation.xy_rotation = 0;
	this->noiseBoxes.push_back(box);
	box = new PeriodicNoiseBox;	box->center.cartesian.x = pw / 2;	box->center.cartesian.y = 17;	box->center.cartesian.z = -1.5;
	box->dimensions.height = 7;	box->dimensions.length = 10;	box->dimensions.width = pw;	box->dynamic_parameters.noise_mean = 0;
	box->dynamic_parameters.noise_variance = 4;	box->static_parameters.noise_period_s = -1;box->rotation.xy_rotation = 0;
	this->noiseBoxes.push_back(box);
	box = new PeriodicNoiseBox;	box->center.cartesian.x =  pw / 2 + 2 + pw;	box->center.cartesian.y = 17;	box->center.cartesian.z = -3.1;
	box->dimensions.height = 7;	box->dimensions.length = 10;	box->dimensions.width = pw;	box->dynamic_parameters.noise_mean = 0;
	box->dynamic_parameters.noise_variance = 1.2;	box->static_parameters.noise_period_s = -1;box->rotation.xy_rotation = 0;
	this->noiseBoxes.push_back(box);

	if (!_sdf->HasElement("bodyName"))
	{
		link = _model->GetLink();
		link_name_ = link->GetName();
	}
	else {
		link_name_ = _sdf->GetElement("bodyName")->GetValue()->GetAsString();
		link = _model->GetLink(link_name_);
	}

	if (!link)
	{
		ROS_FATAL("GazeboRosGpsWithGeofencing plugin error: bodyName: %s does not exist\n", link_name_.c_str());
		return;
	}

	// default parameters
	frame_id_ = "/world";
	fix_topic_ = "terra/fix";
	velocity_topic_ = "terra/fix_velocity";
	terra_topic_ = "terra/gps";

	reference_latitude_  = DEFAULT_REFERENCE_LATITUDE;
	reference_longitude_ = DEFAULT_REFERENCE_LONGITUDE;
	reference_heading_   = DEFAULT_REFERENCE_HEADING * M_PI/180.0;
	reference_altitude_  = DEFAULT_REFERENCE_ALTITUDE;

	fix_.status.status  = sensor_msgs::NavSatStatus::STATUS_FIX;
	fix_.status.service = 0;

	if (_sdf->HasElement("frameId"))
		frame_id_ = _sdf->GetElement("frameId")->GetValue()->GetAsString();

	if (_sdf->HasElement("topicName"))
		fix_topic_ = _sdf->GetElement("topicName")->GetValue()->GetAsString();

	if (_sdf->HasElement("terraTopicName"))
		terra_topic_ = _sdf->GetElement("terraTopicName")->GetValue()->GetAsString();

	if (_sdf->HasElement("velocityTopicName"))
		velocity_topic_ = _sdf->GetElement("velocityTopicName")->GetValue()->GetAsString();

	if (_sdf->HasElement("referenceLatitude"))
		_sdf->GetElement("referenceLatitude")->GetValue()->Get(reference_latitude_);

	if (_sdf->HasElement("referenceLongitude"))
		_sdf->GetElement("referenceLongitude")->GetValue()->Get(reference_longitude_);

	if (_sdf->HasElement("referenceHeading"))
		if (_sdf->GetElement("referenceHeading")->GetValue()->Get(reference_heading_))
			reference_heading_ *= M_PI/180.0;

	if (_sdf->HasElement("referenceAltitude"))
		_sdf->GetElement("referenceAltitude")->GetValue()->Get(reference_altitude_);

	if (_sdf->HasElement("status")) {
		int status = fix_.status.status;
		if (_sdf->GetElement("status")->GetValue()->Get(status))
			fix_.status.status = static_cast<sensor_msgs::NavSatStatus::_status_type>(status);
	}

	if (_sdf->HasElement("service")) {
		unsigned int service = fix_.status.service;
		if (_sdf->GetElement("service")->GetValue()->Get(service))
			fix_.status.service = static_cast<sensor_msgs::NavSatStatus::_service_type>(service);
	}

	fix_.header.frame_id = frame_id_;
	velocity_.header.frame_id = frame_id_;

	position_error_model_.Load(_sdf);
	velocity_error_model_.Load(_sdf, "velocity");

	// calculate earth radii
	double temp = 1.0 / (1.0 - excentrity2 * sin(reference_latitude_ * M_PI/180.0) * sin(reference_latitude_ * M_PI/180.0));
	double prime_vertical_radius = equatorial_radius * sqrt(temp);

	radius_north_ = prime_vertical_radius * (1 - excentrity2) * temp;
	radius_east_  = prime_vertical_radius * cos(reference_latitude_ * M_PI/180.0);

	// Make sure the ROS node for Gazebo has already been initialized
	if (!ros::isInitialized())
	{
		ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
				<< "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
		return;
	}

	node_handle_ = new ros::NodeHandle(namespace_);
	fix_publisher_ = node_handle_->advertise<sensor_msgs::NavSatFix>(fix_topic_, 10);
	velocity_publisher_ = node_handle_->advertise<geometry_msgs::Vector3Stamped>(velocity_topic_, 10);
	terra_publisher_ = node_handle_->advertise<terrasentia_sensors::TerraGps>(terra_topic_, 10);

	// setup dynamic_reconfigure servers
	dynamic_reconfigure_server_position_.reset(new dynamic_reconfigure::Server<SensorModelConfig>(ros::NodeHandle(*node_handle_, fix_topic_ + "/position")));
	dynamic_reconfigure_server_velocity_.reset(new dynamic_reconfigure::Server<SensorModelConfig>(ros::NodeHandle(*node_handle_, fix_topic_ + "/velocity")));
	dynamic_reconfigure_server_status_.reset(new dynamic_reconfigure::Server<GNSSConfig>(ros::NodeHandle(*node_handle_, fix_topic_ + "/status")));
	dynamic_reconfigure_server_position_->setCallback(boost::bind(&SensorModel3::dynamicReconfigureCallback, &position_error_model_, _1, _2));
	dynamic_reconfigure_server_velocity_->setCallback(boost::bind(&SensorModel3::dynamicReconfigureCallback, &velocity_error_model_, _1, _2));
	dynamic_reconfigure_server_status_->setCallback(boost::bind(&GazeboRosGpsWithGeofencing::dynamicReconfigureCallback, this, _1, _2));

	Reset();

	// connect Update function
	updateTimer.setUpdateRate(4.0);
	updateTimer.Load(world, _sdf);
	updateConnection = updateTimer.Connect(boost::bind(&GazeboRosGpsWithGeofencing::Update, this));

}

void GazeboRosGpsWithGeofencing::Reset()
{
	updateTimer.Reset();
	position_error_model_.reset();
	velocity_error_model_.reset();
}

void GazeboRosGpsWithGeofencing::dynamicReconfigureCallback(GazeboRosGpsWithGeofencing::GNSSConfig &config, uint32_t level)
{
	using sensor_msgs::NavSatStatus;
	if (level == 1) {
		if (!config.STATUS_FIX) {
			fix_.status.status = NavSatStatus::STATUS_NO_FIX;
		} else {
			fix_.status.status = (config.STATUS_SBAS_FIX ? NavSatStatus::STATUS_SBAS_FIX : 0) |
					(config.STATUS_GBAS_FIX ? NavSatStatus::STATUS_GBAS_FIX : 0);
		}
		fix_.status.service = (config.SERVICE_GPS     ? NavSatStatus::SERVICE_GPS : 0) |
				(config.SERVICE_GLONASS ? NavSatStatus::SERVICE_GLONASS : 0) |
				(config.SERVICE_COMPASS ? NavSatStatus::SERVICE_COMPASS : 0) |
				(config.SERVICE_GALILEO ? NavSatStatus::SERVICE_GALILEO : 0);
	} else {
		config.STATUS_FIX      = (fix_.status.status != NavSatStatus::STATUS_NO_FIX);
		config.STATUS_SBAS_FIX = (fix_.status.status & NavSatStatus::STATUS_SBAS_FIX);
		config.STATUS_GBAS_FIX = (fix_.status.status & NavSatStatus::STATUS_GBAS_FIX);
		config.SERVICE_GPS     = (fix_.status.service & NavSatStatus::SERVICE_GPS);
		config.SERVICE_GLONASS = (fix_.status.service & NavSatStatus::SERVICE_GLONASS);
		config.SERVICE_COMPASS = (fix_.status.service & NavSatStatus::SERVICE_COMPASS);
		config.SERVICE_GALILEO = (fix_.status.service & NavSatStatus::SERVICE_GALILEO);
	}
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosGpsWithGeofencing::Update()
{
	int i;
	common::Time sim_time = world->GetSimTime();
	double dt = updateTimer.getTimeSinceLastUpdate().Double();

	PeriodicNoiseBox* box = nullptr;


	math::Pose pose = link->GetWorldPose();

	for(i=0;i<this->noiseBoxes.size();i++){
		box = this->noiseBoxes[i];
		if(box->contains((float)pose.pos.x, (float)pose.pos.y, (float)pose.pos.z)){
			ROS_QUICK_INFO("Contained in noise box " << i << "...( p=[" <<  pose.pos.x << " , " << pose.pos.y << "] )\n");

			ROS_QUICK_INFO("GPS noise box for plot: " << i \
					<< " \n [x, y, z], [width, length, height], rotation, [noise-mean, noise-variance] ...\n" \
					<< "[ " << box->center.cartesian.x << "," << box->center.cartesian.y << "," << box->center.cartesian.z << "]\n" \
					<< "[ " << box->dimensions.width << "," << box->dimensions.length << "," << box->dimensions.height << "]\n" \
					<< box->rotation.xy_rotation << "\n" \
					<< "[ " << box->dynamic_parameters.noise_mean << "," << box->dynamic_parameters.noise_variance << "]\n");

			break;
		}
		box = nullptr;
	}
	if(box){
		position_error_model_.gaussian_noise.x *= sqrt(box->dynamic_parameters.noise_variance);
		position_error_model_.gaussian_noise.y *= sqrt(box->dynamic_parameters.noise_variance);
		position_error_model_.gaussian_noise.z *= sqrt(box->dynamic_parameters.noise_variance);
	}

	gazebo::math::Vector3 velocity = velocity_error_model_(link->GetWorldLinearVel(), dt);
	gazebo::math::Vector3 position = position_error_model_(pose.pos, dt);

	// An offset error in the velocity is integrated into the position error for the next timestep.
	// Note: Usually GNSS receivers have almost no drift in the velocity signal.
	position_error_model_.setCurrentDrift(position_error_model_.getCurrentDrift() + dt * velocity_error_model_.getCurrentDrift());

	fix_.header.stamp = ros::Time(sim_time.sec, sim_time.nsec);
	velocity_.header.stamp = fix_.header.stamp;

	fix_.latitude  = reference_latitude_  + ( cos(reference_heading_) * position.x + sin(reference_heading_) * position.y) / radius_north_ * 180.0/M_PI;
	fix_.longitude = reference_longitude_ - (-sin(reference_heading_) * position.x + cos(reference_heading_) * position.y) / radius_east_  * 180.0/M_PI;
	fix_.altitude  = reference_altitude_  + position.z;

	velocity_.vector.x =  cos(reference_heading_) * velocity.x + sin(reference_heading_) * velocity.y;
	velocity_.vector.y = -sin(reference_heading_) * velocity.x + cos(reference_heading_) * velocity.y;
	velocity_.vector.z = velocity.z;

	fix_.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
	fix_.position_covariance[0] = position_error_model_.drift.x*position_error_model_.drift.x + position_error_model_.gaussian_noise.x*position_error_model_.gaussian_noise.x;
	fix_.position_covariance[4] = position_error_model_.drift.y*position_error_model_.drift.y + position_error_model_.gaussian_noise.y*position_error_model_.gaussian_noise.y;
	fix_.position_covariance[8] = position_error_model_.drift.z*position_error_model_.drift.z + position_error_model_.gaussian_noise.z*position_error_model_.gaussian_noise.z;

	gps_m.time = sim_time.Double();
	gps_m.latitude = fix_.latitude;
     gps_m.longitude = fix_.longitude;
     gps_m.altitude = fix_.altitude;
	gps_m.reference_latitude = reference_latitude_;
	gps_m.reference_longitude = reference_longitude_;
	gps_m.reference_altitude = reference_altitude_;
	gps_m.reference_heading = reference_heading_;
     gps_m.velocity.x = velocity_.vector.x;
     gps_m.velocity.y = velocity_.vector.y;
     gps_m.velocity.z = velocity_.vector.z;
     gps_m.covariance.x = fix_.position_covariance[0];
     gps_m.covariance.y = fix_.position_covariance[4];
     gps_m.covariance.z = fix_.position_covariance[8];
     gps_m.covariance_type = fix_.position_covariance_type;
     gps_m.service = fix_.status.status;
     gps_m.status = fix_.status.service;


	fix_publisher_.publish(fix_);
	velocity_publisher_.publish(velocity_);
	terra_publisher_.publish(gps_m);

	this->marker_.color.a = 1.0;
	this->marker_.pose.position.x = position.x;
	this->marker_.pose.position.y = position.y;
	this->marker_.pose.position.z = position.z;
	this->marker_.color.r = 1.0;
	this->marker_.color.g = 1.0;
	this->marker_.color.b = 0.0;
	this->marker_.id = this->measurementCount % 10;
	measurementCount++;

	markerArray_.markers.push_back(this->marker_);
	this->viz_array_pub_.publish(markerArray_);
	markerArray_.markers.clear();


	this->pos_marker_.color.a = 1.0;
	this->pos_marker_.pose.position.x = pose.pos.x;
	this->pos_marker_.pose.position.y = pose.pos.y;
	this->pos_marker_.pose.position.z = pose.pos.z;
	this->pos_marker_.color.r = 0.0;
	this->pos_marker_.color.g = 0.0;
	this->pos_marker_.color.b = 1.0;
	this->pos_marker_.id++;

	markerArray_.markers.push_back(this->pos_marker_);
	this->viz_array_pub_.publish(markerArray_);
	markerArray_.markers.clear();


}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosGpsWithGeofencing)

} // namespace gazebo
