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

#ifndef GAZEBO_SENSOR_COLLECTION_GAZEBO_ROS_GPS_WITH_DROPOUT_H
#define GAZEBO_SENSOR_COLLECTION_GAZEBO_ROS_GPS_WITH_DROPOUT_H

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector>
#include "../sensor_includes.h"
#include <gazebo_sensor_collection/GpsData.h>

namespace gazebo
{

typedef struct POINT{
	float x;
	float y;
}POINT;

typedef struct BiomassDensityGPSNoiseCoefficient{
	struct {
		float min;
		float max;
	}density_range;

	float mean;
	float variance;
}BiomassDensityGPSNoiseCoefficient;

typedef struct BiomassHeightGPSNoiseCoefficient{
	struct {
		float min;
		float max;
	}height_range;

	float mean;
	float variance;
}BiomassHeightGPSNoiseCoefficient;

class FieldNoiseCharacteristics{
public:
	std::vector<BiomassHeightGPSNoiseCoefficient*> biomass_height_gps_noise_coefficients;
	std::vector<BiomassDensityGPSNoiseCoefficient*> biomass_density_gps_noise_coefficients;
};

class PeriodicNoiseBox {
public:

	static POINT rotate_point(float cx,float cy,float angle,POINT p)
	{
		float s = sin(angle);
		float c = cos(angle);

		// translate point back to origin:
		p.x -= cx;
		p.y -= cy;

		// rotate point
		float xnew = p.x * c - p.y * s;
		float ynew = p.x * s + p.y * c;

		// translate point back:
		p.x = xnew + cx;
		p.y = ynew + cy;
		return p;
	}

	float top(){
		return this->center.cartesian.z + this->dimensions.height /2;
	}
	float bottom(){
		return this->center.cartesian.z - this->dimensions.height /2;
	}

	void update(float time_s){

	};
	bool contains(float x, float y, float z);
	bool contains(double lat, double lon, float alt){

	};



	struct {
		union{
			struct{
				double lat;
				double lon;
				float alt;
			} geo;
			struct{
				double x;
				double y;
				float z;
			} cartesian;
		};
	}center;

	struct{
		union{
			float ne_rotation;
			float xy_rotation;
		};
	}rotation;

	struct{
		float length;
		float width;
		float height; // -1 indicates infinite height
	}dimensions;

	//every noise period, noise_variance will change from noise_variance_periodic_mean according to noise_variance_perioidic_variance
	// every noise period, noise_mean will change from noise_mean_periodic_mean according to noise_mean_periodic_variance;
	// every sampling period, the state.noise will change from noise_mean according to noise_variance
	struct{
		float noise_period_s; // -1 indicates permanent
		double noise_mean_periodic_mean;
		double noise_mean_periodic_variance;
		double noise_variance_periodic_mean;
		double noise_variance_perioidic_variance;
	}static_parameters;

	struct{
		double noise_mean;
		double noise_variance;
	}dynamic_parameters;

	struct{
		double noise;
		float update_time_s;
	} state;
};

class GazeboRosGpsWithGeofencing : public ModelPlugin
{
public:
	GazeboRosGpsWithGeofencing();
	virtual ~GazeboRosGpsWithGeofencing();

protected:
	virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
	virtual void Reset();
	virtual void Update();

	typedef hector_gazebo_plugins::GNSSConfig GNSSConfig;
	void dynamicReconfigureCallback(GNSSConfig &config, uint32_t level);

private:
	FieldNoiseCharacteristics field_noise_characteristics;
	std::vector<PeriodicNoiseBox*> noiseBoxes;
	/// \brief The parent World
	physics::WorldPtr world;

	/// \brief The link referred to by this plugin
	physics::LinkPtr link;

	ros::NodeHandle* node_handle_;
	ros::Publisher fix_publisher_;
	ros::Publisher velocity_publisher_;
	ros::Publisher data_publisher_;

	sensor_msgs::NavSatFix fix_;
	gazebo_sensor_collection::GpsData gps_m;
	geometry_msgs::Vector3Stamped velocity_;

	std::string namespace_;
	std::string link_name_;
	std::string frame_id_;
	std::string fix_topic_;
	std::string velocity_topic_;
	std::string data_topic_;

	double reference_latitude_;
	double reference_longitude_;
	double reference_heading_;
	double reference_altitude_;

	double radius_north_;
	double radius_east_;

	SensorModel3 position_error_model_;
	SensorModel3 velocity_error_model_;

	UpdateTimer updateTimer;
	event::ConnectionPtr updateConnection;

	boost::shared_ptr<dynamic_reconfigure::Server<SensorModelConfig> > dynamic_reconfigure_server_position_, dynamic_reconfigure_server_velocity_;
	boost::shared_ptr<dynamic_reconfigure::Server<GNSSConfig> > dynamic_reconfigure_server_status_;

	ros::Publisher viz_array_pub_;

	visualization_msgs::Marker marker_;
	visualization_msgs::Marker pos_marker_;
	visualization_msgs::MarkerArray markerArray_;
	int measurementCount;
	ros::NodeHandle private_nh_;
	ros::NodeHandle nh_;
};

} // namespace gazebo

#endif // GAZEBO_SENSOR_COLLECTION_GAZEBO_ROS_GPS_WITH_DROPOUT_H
