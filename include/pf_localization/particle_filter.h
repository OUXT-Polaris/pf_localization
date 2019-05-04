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

//headers in STL
#include <map>

struct Particle
{
    geometry_msgs::PoseStamped pose;
    double weight;
};

class ParticleFilter
{
public:
    ParticleFilter(int num_particles,double buffer_length);
    ~ParticleFilter();
    const int num_particles;
    const double buffer_length;
    void updateTwist(std::string key,double weight,geometry_msgs::TwistStamped twist);
    void updatePoint(std::string key,double weight,geometry_msgs::PointStamped point);
    boost::optional<geometry_msgs::PoseStamped> estimatePose(ros::Time stamp);
    void setInitialPose(geometry_msgs::PoseStamped pose);
private:
    DataBuffer buf_;
    std::map<std::string,double> twist_weights_;
    std::map<std::string,double> point_weights_;
    boost::optional<geometry_msgs::TwistStamped> estimateTwist(ros::Time stamp);
    boost::optional<geometry_msgs::PointStamped> estimatePoint(ros::Time stamp);
    std::vector<Particle> particles_;
    bool initialized;
};

#endif  //PF_LOCALIZATION_PARTICLE_FILTER_H_INCLUDED