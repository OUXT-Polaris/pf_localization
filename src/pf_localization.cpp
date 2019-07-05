#include <pf_localization/pf_localization.h>

PfLocalization::PfLocalization(ros::NodeHandle nh,ros::NodeHandle pnh) : nh_(nh),pnh_(pnh), tf_listener_(tf_buffer_)
{
    pose_recieved_ = false;
    pnh_.param<int>("num_particles", num_particles_, 100);
    pnh_.param<int>("update_rate", update_rate_, 20);
    pnh_.param<std::string>("pose_topic", pose_topic_, "/gps/fix/position");
    pnh_.param<std::string>("twist_topic", twist_topic_, "/twist");
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    pnh_.param<std::string>("base_link_frame", base_link_frame_, "base_link");
    pnh_.param<bool>("estimate_3d_pose",estimate_3d_pose_,false);
    pnh_.param<bool>("publish_marker",publish_marker_,false);
    pnh_.param<double>("reset_ess_threashold",reset_ess_threashold_,0.5);
    pnh_.param<double>("max_expantion_orientation",max_expantion_orientation_,0.5);
    pnh_.param<double>("max_expantion_position",max_expantion_position_,0.5);
    pf_ptr_ = std::make_shared<ParticleFilter>(num_particles_,1,estimate_3d_pose_,base_link_frame_,
        reset_ess_threashold_,max_expantion_orientation_,max_expantion_position_);
    current_pose_pub_ = pnh_.advertise<geometry_msgs::PoseStamped>("current_pose",1);
    current_twist_pub_ = pnh_.advertise<geometry_msgs::TwistStamped>("current_twist",1);
    if(publish_marker_)
    {
        marker_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("marker",1);
    }
    twist_sub_ = nh_.subscribe(twist_topic_,1,&PfLocalization::twistStampedCallback,this);
    pose_sub_ = nh_.subscribe(pose_topic_,1,&PfLocalization::poseStampedCallback,this);
}

PfLocalization::~PfLocalization()
{

}

void PfLocalization::run()
{
    boost::thread pose_update_thread(boost::bind(&PfLocalization::updateCurrentPose, this));
    return;
}

boost::optional<geometry_msgs::TwistStamped> PfLocalization::getCurrentTwist()
{
    return pf_ptr_->getCurrentTwist();
}

void PfLocalization::updateCurrentPose()
{
    ros::Rate rate(update_rate_);
    while(ros::ok())
    {
        mtx_.lock();
        ros::Time now = ros::Time::now();
        boost::optional<geometry_msgs::PoseStamped> current_pose = pf_ptr_->estimateCurrentPose(now);
        if(current_pose)
        {
            broadcastInitialPoseFrame(now);
            broadcastBaseLinkFrame(now,*current_pose);
            current_pose_pub_.publish(*current_pose);
        }
        if(publish_marker_)
        {
            visualization_msgs::MarkerArray marker;
            std::vector<Particle> particles = pf_ptr_->getParticles();
            int id = 0;
            for(auto itr = particles.begin(); itr != particles.end(); itr++)
            {
                visualization_msgs::Marker single_marker;
                single_marker.header = itr->pose.header;
                single_marker.pose = itr->pose.pose;
                single_marker.type = single_marker.ARROW;
                single_marker.id = id;
                single_marker.color.r = 1.0-itr->weight;
                single_marker.color.g = 0.0;
                single_marker.color.b = itr->weight;
                single_marker.color.a = 1.0;
                single_marker.action = single_marker.ADD;
                single_marker.frame_locked = true;
                single_marker.scale.x = 1.0;
                single_marker.scale.y = 0.1;
                single_marker.scale.z = 0.1;
                single_marker.ns = "marker" + std::to_string(id);
                marker.markers.push_back(single_marker);
                id++;
            }
            marker_pub_.publish(marker);
        }
        boost::optional<geometry_msgs::TwistStamped> current_twist = getCurrentTwist();
        if(current_twist)
        {
            current_twist_pub_.publish(*current_twist);
        }
        mtx_.unlock();
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

void PfLocalization::broadcastInitialPoseFrame(ros::Time stamp)
{
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.frame_id = map_frame_;
    //transform_stamped.header.stamp = stamp;
    transform_stamped.child_frame_id = "initial_pose";
    transform_stamped.transform.translation.x = initial_pose_.pose.position.x;
    transform_stamped.transform.translation.y = initial_pose_.pose.position.y;
    transform_stamped.transform.translation.z = initial_pose_.pose.position.z;
    transform_stamped.transform.rotation.x = initial_pose_.pose.orientation.x;
    transform_stamped.transform.rotation.y = initial_pose_.pose.orientation.y;
    transform_stamped.transform.rotation.z = initial_pose_.pose.orientation.z;
    transform_stamped.transform.rotation.w = initial_pose_.pose.orientation.w;
    static_tf_broadcaster_.sendTransform(transform_stamped);
    return;
}

void PfLocalization::twistStampedCallback(const geometry_msgs::TwistStamped::ConstPtr msg)
{
    mtx_.lock();
    pf_ptr_->updateTwist(*msg);
    mtx_.unlock();
    return;
}

void PfLocalization::poseStampedCallback(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    mtx_.lock();
    if(!pose_recieved_)
    {
        initial_pose_ = *msg;
        pf_ptr_->setInitialPose(*msg);
        pose_recieved_ = true;
    }
    pf_ptr_->updatePose(*msg);
    mtx_.unlock();
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
            ROS_ERROR("%s",ex.what());
            return boost::none;
        }
        tf2::doTransform(input,input,transform_stamped);
    }
    return input;
}