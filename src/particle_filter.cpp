#include <pf_localization/particle_filter.h>

#include <quaternion_operation/quaternion_operation.h>

ParticleFilter::ParticleFilter(int num_particles,double buffer_length,bool estimate_3d_pose) 
    : num_particles(num_particles),buffer_length(buffer_length),buf_(buffer_length),estimate_3d_pose(estimate_3d_pose),
    engine_(seed_gen_()),dist_(1.0,0.01),mt_(seed_gen_()),uniform_dist_(0.0,1.0)
{
    particles_ = std::vector<Particle>(num_particles);
    current_pose_ = boost::none;
    initial_pose_ = boost::none;
}

ParticleFilter::~ParticleFilter()
{

}

boost::optional<geometry_msgs::PoseStamped> ParticleFilter::getInitialPose()
{
    return initial_pose_;
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
    current_pose_ = pose;
    initial_pose_ = pose;
    for(auto itr = particles_.begin(); itr != particles_.end(); itr++)
    {
        itr->weight = (double)1.0/num_particles;
        itr->pose = pose;
    }
    return;
}

boost::optional<geometry_msgs::PoseStamped> ParticleFilter::estimatePose(ros::Time stamp)
{
    //boost::optional<geometry_msgs::TwistStamped> twist = estimateTwist(stamp);
    boost::optional<geometry_msgs::PointStamped> point = estimatePoint(stamp); // Infinite Loop contains
    /*
    if(twist && point && current_pose_)
    {
        double duration = (stamp - current_pose_->header.stamp).toSec();
        for(auto itr = particles_.begin(); itr != particles_.end(); itr++)
        {
            // Transition
            geometry_msgs::Vector3 orientation;
            if(estimate_3d_pose)
            {
                orientation.x = twist->twist.angular.x * duration * dist_(engine_);
                orientation.y = twist->twist.angular.y * duration * dist_(engine_);
            }
            orientation.z = twist->twist.angular.z * duration * dist_(engine_);
            geometry_msgs::Quaternion twist_angular_quat = 
                quaternion_operation::convertEulerAngleToQuaternion(orientation);
            itr->pose.pose.orientation = quaternion_operation::rotation(itr->pose.pose.orientation,twist_angular_quat);
            if(estimate_3d_pose)
            {
                itr->pose.pose.position.z = itr->pose.pose.position.z + twist->twist.linear.z * duration * dist_(engine_);
            }
            itr->pose.pose.position.x = itr->pose.pose.position.x + twist->twist.linear.x * duration * dist_(engine_);
            itr->pose.pose.position.y = itr->pose.pose.position.y + twist->twist.linear.y * duration * dist_(engine_);
            // Evaluate
            double dist = std::sqrt(std::pow(itr->pose.pose.position.x-point->point.x,2)
                + std::pow(itr->pose.pose.position.y-point->point.y,2)
                + std::pow(itr->pose.pose.position.z-point->point.z,2));
            if(dist < 0.0001)
            {
                dist = 0.0001;
            }
            itr->weight = 1/dist;
        }
        double total_weight = 0.0;
        double heighest_weight = 0;
        geometry_msgs::PoseStamped ret;
        for(auto itr = particles_.begin(); itr != particles_.end(); itr++)
        {
            total_weight = total_weight + itr->weight;
            if(heighest_weight < itr->weight)
            {
                heighest_weight = itr->weight;
                ret = itr->pose;
            }
        }
        // Resampling
        std::vector<int> selected_index = std::vector<int>(particles_.size());
        double init_value = uniform_dist_(mt_) * total_weight;
        int current_index = 0;
        double current_total_weight = 0.0;
        ROS_ERROR_STREAM(particles_.size());
        for(int i=0; i<particles_.size(); i++)
        {
            current_total_weight = current_total_weight + particles_[i].weight;
            if((init_value + total_weight/(double)particles_.size()*current_index) < current_total_weight)
            {
                selected_index[current_index] = i;
                current_index++;
            }
        }
        std::vector<Particle> new_particles = std::vector<Particle>(particles_.size());
        for(int i=0; i<particles_.size(); i++)
        {
            new_particles[i] = particles_[selected_index[i]];
        }
        particles_ = new_particles;
        current_pose_ = ret;
        return ret;
    }
    */
    ROS_ERROR_STREAM("return none");
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