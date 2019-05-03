#ifndef PF_LOCALIZATION_PARTICLE_FILTER_H_INCLUDED
#define PF_LOCALIZATION_PARTICLE_FILTER_H_INCLUDED

//headers in this package
#include <pf_localization/data_buffer.h>

//headers in ROS
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

//headers in Boost
#include <boost/optional.hpp>

class ParticleFilter
{
public:
    ParticleFilter(int num_particles,double buffer_length);
    ~ParticleFilter();
    const int num_particles;
    const double buffer_length;
    void updateTwist(std::string key,geometry_msgs::TwistStamped twist);
    void updatePoint(std::string key,geometry_msgs::PointStamped point);
    boost::optional<geometry_msgs::PoseStamped> estimatePose(ros::Time stamp);
private:
    DataBuffer buf_;
};

#endif  //PF_LOCALIZATION_PARTICLE_FILTER_H_INCLUDED