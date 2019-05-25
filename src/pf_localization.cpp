#include <pf_localization/pf_localization.h>

PfLocalization::PfLocalization(ros::NodeHandle nh,ros::NodeHandle pnh) : nh_(nh),pnh_(pnh), tf_listener_(tf_buffer_)
{
    pnh_.param<int>("num_particles", num_particles_, 100);
    pnh_.param<int>("update_rate", update_rate_, 30);
    pnh_.param<std::string>("position_topic", position_topic_, "/gps/fix/position");
    pnh_.param<std::string>("twist_topic", twist_topic_, "/twist");
    pnh_.param<std::string>("initial_pose_topic", initial_pose_topic_, "/initialpose");
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    pnh_.param<std::string>("base_link_frame", base_link_frame_, "base_link");
    pnh_.param<bool>("use_2d_pose_estimate",use_2d_pose_estimate_,false);
    pf_ptr_ = std::make_shared<ParticleFilter>(num_particles_,10);
    current_pose_pub_ = pnh_.advertise<geometry_msgs::PoseStamped>("current_pose",1);
    twist_sub_ = nh_.subscribe(twist_topic_,1,&PfLocalization::twistStampedCallback,this);
    point_sub_ = nh_.subscribe(position_topic_,1,&PfLocalization::pointStampedCallback,this);
    if(use_2d_pose_estimate_)
    {
        initial_pose_sub_ = nh_.subscribe(initial_pose_topic_,1,&PfLocalization::initialPoseCallback,this);
    }
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
            broadcastBaseLinkFrame(now,*current_pose);
            current_pose_pub_.publish(*current_pose);
        }
        rate.sleep();
    }
    return;
}

void PfLocalization::broadcastBaseLinkFrame(ros::Time stamp,geometry_msgs::PoseStamped pose)
{
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.frame_id = map_frame_;
    transform_stamped.header.stamp = stamp;
    transform_stamped.child_frame_id = base_link_frame_;
    transform_stamped.transform.translation.x = pose.pose.position.x;
    transform_stamped.transform.translation.y = pose.pose.position.y;
    transform_stamped.transform.translation.z = pose.pose.position.z;
    transform_stamped.transform.rotation = pose.pose.orientation;
    tf_broadcaster_.sendTransform(transform_stamped);
    return;
}

void PfLocalization::twistStampedCallback(const geometry_msgs::TwistStamped::ConstPtr msg)
{
    pf_ptr_->updateTwist(position_topic_,1,*msg);
    return;
}

void PfLocalization::pointStampedCallback(const geometry_msgs::PointStamped::ConstPtr msg)
{
    pf_ptr_->updatePoint(twist_topic_,1,*msg);
    return;
}

template <class C>
boost::optional<C> PfLocalization::transformToMapFrame(C input)
{
    if(input.header.frame_id != map_frame_)
    {
        geometry_msgs::TransformStamped transform_stamped;
        try
        {
            transform_stamped = tf_buffer_.lookupTransform(map_frame_, input.header.frame_id, input.header.stamp, ros::Duration(0.3));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            return boost::none;
        }
        tf2::doTransform(input,input,transform_stamped);
    }
    return input;
}

void PfLocalization::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg)
{
    geometry_msgs::PoseStamped pose;
    pose.header = msg->header;
    pose.pose = msg->pose.pose;
    boost::optional<geometry_msgs::PoseStamped> pose_transformed = transformToMapFrame(pose);
    if(pose_transformed)
    {
        pf_ptr_->setInitialPose(*pose_transformed);
    }
    return;
}