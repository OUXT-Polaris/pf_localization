#include <pf_localization/particle_filter.h>

ParticleFilter::ParticleFilter(int num_particles,double buffer_length) 
    : num_particles(num_particles),buffer_length(buffer_length),buf_(buffer_length)
{
    particles_ = std::vector<Particle>(num_particles);
    initialized = false;
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

void ParticleFilter::setInitialPose(geometry_msgs::PoseStamped pose)
{
    initialized = true;
    for(auto itr = particles_.begin(); itr != particles_.end(); itr++)
    {
        itr->weight = (double)1.0/num_particles;
        itr->pose = pose;
    }
    return;
}

boost::optional<geometry_msgs::PoseStamped> ParticleFilter::estimatePose(ros::Time stamp)
{
    boost::optional<geometry_msgs::TwistStamped> twist = estimateTwist(stamp);
    boost::optional<geometry_msgs::PointStamped> point = estimatePoint(stamp);
    if(twist && point && initialized)
    {
        
    }
    return boost::none;
}

boost::optional<geometry_msgs::TwistStamped> ParticleFilter::estimateTwist(ros::Time stamp)
{
    std::map<std::string,std::pair<double,geometry_msgs::TwistStamped> > data;
    double total_weight = 0;
    for(auto itr = twist_weights_.begin(); itr != twist_weights_.end(); itr++)
    {
        geometry_msgs::TwistStamped twist;
        if(buf_.queryData(stamp,itr->first,twist))
        {
            std::pair<double,geometry_msgs::TwistStamped> pair;
            total_weight = total_weight + itr->second;
            pair.first = itr->second;
            pair.second = twist;
            data[itr->first] = pair;
        }
    }
    if(data.size() == 0)
    {
        return boost::none;
    }
    for(auto itr = data.begin(); itr != data.end(); itr++)
    {
        itr->second.first = itr->second.first/total_weight;
    }
    geometry_msgs::TwistStamped ret;
    for(auto itr = data.begin(); itr != data.end(); itr++)
    {
        ret.twist.linear.x = ret.twist.linear.x + itr->second.second.twist.linear.x * itr->second.first;
        ret.twist.linear.y = ret.twist.linear.y + itr->second.second.twist.linear.y * itr->second.first;
        ret.twist.linear.z = ret.twist.linear.z + itr->second.second.twist.linear.z * itr->second.first;
        ret.twist.angular.x = ret.twist.angular.x + itr->second.second.twist.angular.x * itr->second.first;
        ret.twist.angular.y = ret.twist.angular.y + itr->second.second.twist.angular.y * itr->second.first;
        ret.twist.angular.z = ret.twist.angular.z + itr->second.second.twist.angular.z * itr->second.first;
    }
    return ret;
}

boost::optional<geometry_msgs::PointStamped> ParticleFilter::estimatePoint(ros::Time stamp)
{
    std::map<std::string,std::pair<double,geometry_msgs::PointStamped> > data;
    double total_weight = 0;
    for(auto itr = point_weights_.begin(); itr != point_weights_.end(); itr++)
    {
        geometry_msgs::PointStamped point;
        if(buf_.queryData(stamp,itr->first,point))
        {
            std::pair<double,geometry_msgs::PointStamped> pair;
            total_weight = total_weight + itr->second;
            pair.first = itr->second;
            pair.second = point;
            data[itr->first] = pair;
        }
    }
    if(data.size() == 0)
    {
        return boost::none;
    }
    for(auto itr = data.begin(); itr != data.end(); itr++)
    {
        itr->second.first = itr->second.first/total_weight;
    }
    geometry_msgs::PointStamped ret;
    for(auto itr = data.begin(); itr != data.end(); itr++)
    {
        ret.point.x = ret.point.x + itr->second.second.point.x * itr->second.first;
        ret.point.y = ret.point.y + itr->second.second.point.y * itr->second.first;
        ret.point.z = ret.point.z + itr->second.second.point.z * itr->second.first;
    }
    return ret;
}