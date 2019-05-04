#include <pf_localization/particle_filter.h>

ParticleFilter::ParticleFilter(int num_particles,double buffer_length) 
    : num_particles(num_particles),buffer_length(buffer_length),buf_(buffer_length)
{
}

ParticleFilter::~ParticleFilter()
{

}

void ParticleFilter::updateTwist(std::string key,double weight,geometry_msgs::TwistStamped twist)
{
    buf_.addData(key,twist);
    twist_weights_[key] = weight;
    return;
}

void ParticleFilter::updatePoint(std::string key,double weight,geometry_msgs::PointStamped point)
{
    buf_.addData(key,point);
    point_weights_[key] = weight;
    return;
}

boost::optional<geometry_msgs::PoseStamped> ParticleFilter::estimatePose(ros::Time stamp)
{
    geometry_msgs::TwistStamped twist;
    geometry_msgs::PointStamped point;
    //if(buf_.queryData(,twist))
    return boost::none;
}

boost::optional<geometry_msgs::TwistStamped> ParticleFilter::estimateTwist(ros::Time stamp)
{
    return boost::none;
}

boost::optional<geometry_msgs::PointStamped> ParticleFilter::estimatePoint(ros::Time stamp)
{
    return boost::none;
}