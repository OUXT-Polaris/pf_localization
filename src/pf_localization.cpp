#include <pf_localization/pf_localization.h>

PfLocalization::PfLocalization(ros::NodeHandle nh,ros::NodeHandle pnh) : nh_(nh),pnh_(pnh)
{
    pnh_.param<int>("num_particles", num_particles_, 100);
    pnh_.param<std::string>("fix_position_topic", fix_position_topic_, "/gps/fix/position");
    pnh_.param<std::string>("twist_topic", twist_topic_, "/twist");
    pf_ptr_ = std::make_shared<ParticleFilter>(num_particles_,10);
}

PfLocalization::~PfLocalization()
{

}

void PfLocalization::twistStampedCallback(const geometry_msgs::TwistStamped::ConstPtr msg)
{
    pf_ptr_->updateTwist(fix_position_topic_,*msg);
    return;
}

void PfLocalization::pointStampedCallback(const geometry_msgs::PointStamped::ConstPtr msg)
{
    pf_ptr_->updatePoint(twist_topic_,*msg);
    return;
}