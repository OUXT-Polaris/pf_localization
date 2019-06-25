#include <pf_localization/particle_filter.h>

#include <quaternion_operation/quaternion_operation.h>

ParticleFilter::ParticleFilter(int num_particles,double buffer_length,bool estimate_3d_pose) 
    : num_particles(num_particles),buffer_length(buffer_length),estimate_3d_pose(estimate_3d_pose),
    engine_(seed_gen_()),dist_(1.0,0.1),rotation_dist_(1.0,0.1),mt_(seed_gen_()),uniform_dist_(0.0,1.0),
    pose_buf_("/pose",buffer_length),twist_buf_("/twist",buffer_length)
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

void ParticleFilter::updateTwist(geometry_msgs::TwistStamped twist)
{
    twist_buf_.addData(twist);
}

void ParticleFilter::updatePose(geometry_msgs::PoseStamped pose)
{
    pose_buf_.addData(pose);
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

bool ParticleFilter::checkQuaternion(geometry_msgs::Quaternion quat)
{
    double a = std::sqrt(std::pow(quat.x,2) + std::pow(quat.y,2) + std::pow(quat.z,2) + std::pow(quat.w,2));
    double b = 1.0;
    if (fabs(a - b) < DBL_EPSILON)
    {
        return true;
    }
    return false;
}

boost::optional<geometry_msgs::PoseStamped> ParticleFilter::estimateCurrentPose(ros::Time stamp)
{
    geometry_msgs::TwistStamped twist;
    bool twist_query_succeed = twist_buf_.queryData(stamp,twist);
    geometry_msgs::PoseStamped pose;
    bool pose_query_succeed = pose_buf_.queryData(stamp,pose);
    if(twist_query_succeed && pose_query_succeed && current_pose_)
    {
        double duration = (stamp - current_pose_->header.stamp).toSec();
        for(auto itr = particles_.begin(); itr != particles_.end(); itr++)
        {
            // Transition
            geometry_msgs::Vector3 orientation;
            if(estimate_3d_pose)
            {
                orientation.x = twist.twist.angular.x * duration * rotation_dist_(engine_);
                orientation.y = twist.twist.angular.y * duration * rotation_dist_(engine_);
            }
            else
            {
                orientation.x = 0.0;
                orientation.y = 0.0;
            }
            orientation.z = twist.twist.angular.z * duration * rotation_dist_(engine_);
            //ROS_ERROR_STREAM(orientation);
            geometry_msgs::Quaternion twist_angular_quat = 
                quaternion_operation::convertEulerAngleToQuaternion(orientation);
            //ROS_ERROR_STREAM(twist_angular_quat);
            itr->pose.pose.orientation = quaternion_operation::rotation(itr->pose.pose.orientation,twist_angular_quat);
            //ROS_WARN_STREAM(itr->pose.pose.orientation);
            Eigen::Vector3d trans_vec;
            if(estimate_3d_pose)
            {
                //itr->pose.pose.position.z = itr->pose.pose.position.z + twist.twist.linear.z * duration * dist_(engine_);
                trans_vec(0) = twist.twist.linear.z * duration * dist_(engine_);
            }
            else
            {
                //itr->pose.pose.position.z = 0;
                trans_vec(0)  = 0;
            }
            trans_vec(1) = twist.twist.linear.x * duration * dist_(engine_);
            trans_vec(2) = twist.twist.linear.y * duration * dist_(engine_);
            Eigen::Matrix3d rotation_mat = quaternion_operation::getRotationMatrix(itr->pose.pose.orientation);
            trans_vec = rotation_mat*trans_vec;
            itr->pose.pose.position.x = itr->pose.pose.position.x + trans_vec(0);
            itr->pose.pose.position.y = itr->pose.pose.position.y + trans_vec(1);
            if(estimate_3d_pose)
            {
                itr->pose.pose.position.z = itr->pose.pose.position.z + trans_vec(2);
            }
            //itr->pose.pose.position.x = itr->pose.pose.position.x + twist.twist.linear.x * duration * dist_(engine_);
            //itr->pose.pose.position.y = itr->pose.pose.position.y + twist.twist.linear.y * duration * dist_(engine_);
            // Evaluate
            double dist = std::sqrt(std::pow(itr->pose.pose.position.x-pose.pose.position.x,2)
                + std::pow(itr->pose.pose.position.y-pose.pose.position.y,2)
                + std::pow(itr->pose.pose.position.z-pose.pose.position.z,2));
            geometry_msgs::Quaternion diff_quat = quaternion_operation::getRotation(itr->pose.pose.orientation,pose.pose.orientation);
            geometry_msgs::Vector3 diff_vec = quaternion_operation::convertQuaternionToEulerAngle(diff_quat);
            double diff_angle = std::sqrt(diff_vec.x*diff_vec.x + diff_vec.y*diff_vec.y + diff_vec.z*diff_vec.z);
            // avoid zero diveide
            if(dist < 0.0001)
            {
                dist = 0.0001;
            }
            if(diff_angle < 0.0001)
            {
                diff_angle = 0.0001;
            }
            itr->weight = 1/(dist*diff_angle);
        }
        double total_weight = 0.0;
        double heighest_weight = 0;
        geometry_msgs::PoseStamped ret = particles_[0].pose;
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
        ret.header.stamp = stamp;
        current_pose_ = ret;
        return ret;
    }
    return boost::none;
}