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
#ifndef CUSTOM_GAZEBO_PLUGINS_GNSS_CONFIG_H
#define CUSTOM_GAZEBO_PLUGINS_GNSS_CONFIG_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <gazebo_ros/node.hpp>

namespace gazebo{

class GNSSConfig{
public:
    GNSSConfig(gazebo_ros::Node::SharedPtr node){
        this->node_ = node;
        // Initialize the parameters
        node_->declare_parameter<bool>("status_fix", true);  // unaugmented fix
        node_->declare_parameter<bool>("status_sbas_fix", false);  // fix with satellite-based augmentation
        node_->declare_parameter<bool>("status_gbas_fix", false);  // with ground-based augmentation
        node_->declare_parameter<bool>("service_gps", true);  // GPS service
        node_->declare_parameter<bool>("service_glonass", true);  // GLONASS service
        node_->declare_parameter<bool>("service_compass", true);  // COMPASS service
        node_->declare_parameter<bool>("service_galileo", true);  // GALILEO service
    }
    virtual ~GNSSConfig(){ node_ = nullptr; }
private:
    gazebo_ros::Node::SharedPtr node_;
};

// SECTION: Stuff for defining GPS Noise Areas (aka patches where GPS accuracy drops)
typedef struct Point2p{
    float x;
    float y;
}Point2p;
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
    static Point2p rotate_point(float cx,float cy,float angle,Point2p p){
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

    float top(){ return this->center.cartesian.z + this->dimensions.height /2; }
    float bottom(){ return this->center.cartesian.z - this->dimensions.height /2; }
    void update(float time_s){};
    bool contains(float x, float y, float z){
        if(this->top() > z || this->bottom() < z){ return false; }

        Point2p p;
        p.x = x;
        p.y = y;
        Point2p rot = rotate_point(this->center.cartesian.x, this->center.cartesian.y, -this->rotation.xy_rotation, p);

        //ROS_QUICK_INFO("rx, ry ... " << rot.x << ", " << rot.y);
        if(rot.x > this->center.cartesian.x + this->dimensions.width / 2){ return false; }
        if(rot.x < this->center.cartesian.x - this->dimensions.width / 2){ return false; }
        if(rot.y > this->center.cartesian.y + this->dimensions.length / 2){ return false; }
        if(rot.y < this->center.cartesian.y - this->dimensions.length / 2){ return false; }
        return true;
    };
    bool contains(double lat, double lon, float alt){};

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


//END  SECTION: Stuff for defining GPS Noise Areas (aka patches where GPS accuracy drops)

} // namespace gazebo

#endif // CUSTOM_GAZEBO_PLUGINS_GNSS_CONFIG_H
