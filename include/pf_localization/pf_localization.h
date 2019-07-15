#ifndef PF_LOCALIZATION_PF_LOCALIZATION_H_INCLUDED
#define PF_LOCALIZATION_PF_LOCALIZATION_H_INCLUDED

// Headers in this package
#include <pf_localization/particle_filter.h>

// Headers in ROS
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Headers in STL
#include <memory>
#include <mutex>

// Headers in Boost
#include <boost/optional.hpp>
#include <boost/thread.hpp>

class PfLocalization
{
public:
    PfLocalization(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~PfLocalization();
    void run();
private:
    void updateCurrentPose();
    boost::optional<geometry_msgs::TwistStamped> getCurrentTwist();
    void twistStampedCallback(const geometry_msgs::TwistStamped::ConstPtr msg);
    void poseStampedCallback(const geometry_msgs::PoseStamped::ConstPtr msg);
    std::mutex mtx_;
    std::shared_ptr<ParticleFilter> pf_ptr_;
    int num_particles_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::string pose_topic_;
    std::string twist_topic_;
    std::string map_frame_;
    std::string base_link_frame_;
    int update_rate_;
    ros::Publisher current_pose_pub_;
    ros::Subscriber twist_sub_;
    ros::Subscriber pose_sub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    bool pose_recieved_;
    bool estimate_3d_pose_;
    double expansion_reset_ess_threashold_;
    double max_expantion_orientation_;
    double max_expantion_position_;
    double sensor_reset_ess_threashold_;
    double max_sensor_reset_orientation_;
    double max_sensor_reset_position_;
    double sensor_reset_radius_;
    void broadcastBaseLinkFrame(ros::Time stamp,geometry_msgs::PoseStamped pose);
    void broadcastInitialPoseFrame(ros::Time stamp);
    template <class C>
    boost::optional<C> transformToMapFrame(C input);
    ros::Publisher current_twist_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher ess_pub_;
    bool publish_marker_;
    geometry_msgs::PoseStamped initial_pose_;
};

#endif  //PF_LOCALIZATION_PF_LOCALIZATION_H_INCLUDED