#include <pf_localization/pf_localization.h>

PfLocalization::PfLocalization(ros::NodeHandle nh,ros::NodeHandle pnh) : nh_(nh),pnh_(pnh)
{
    pnh_.param<int>("num_particles", num_particles_, 100);
    pnh_.param<int>("update_rate", update_rate_, 30);
    pnh_.param<std::string>("fix_position_topic", fix_position_topic_, "/gps/fix/position");
    pnh_.param<std::string>("twist_topic", twist_topic_, "/twist");
    pf_ptr_ = std::make_shared<ParticleFilter>(num_particles_,10);
    current_pose_pub_ = pnh_.advertise<geometry_msgs::PoseStamped>("current_pose",1);
    twist_sub_ = nh_.subscribe(twist_topic_,1,&PfLocalization::twistStampedCallback,this);
    point_sub_ = nh_.subscribe(fix_position_topic_,1,&PfLocalization::pointStampedCallback,this);
}

PfLocalization::~PfLocalization()
{

}

void PfLocalization::run()
{
    boost::thread pose_update_thread(boost::bind(&PfLocalization::updateCurrentPose, this));
    return;
}

void PfLocalization::updateCurrentPose()
{
    ros::Rate rate(update_rate_);
    while(ros::ok())
    {
        ros::Time now = ros::Time::now();
        boost::optional<geometry_msgs::PoseStamped> current_pose = pf_ptr_->estimatePose(now);
        if(current_pose)
        {
            current_pose_pub_.publish(*current_pose);
        }
        rate.sleep();
    }
    return;
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