#include <pf_localization/particle_filter.h>

ParticleFilter::ParticleFilter(int num_particles,double buffer_length) 
    : num_particles(num_particles),buffer_length(buffer_length),buf_(buffer_length)
{
}

ParticleFilter::~ParticleFilter()
{

}

void ParticleFilter::updateTwist(std::string key,geometry_msgs::TwistStamped twist)
{
    buf_.addData(key,twist);
    return;
}

void ParticleFilter::updatePoint(std::string key,geometry_msgs::PointStamped point)
{
    buf_.addData(key,point);
    return;
}

boost::optional<geometry_msgs::PoseStamped> ParticleFilter::estimatePose(ros::Time stamp)
{

}