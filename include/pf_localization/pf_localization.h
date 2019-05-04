#ifndef PF_LOCALIZATION_PF_LOCALIZATION_H_INCLUDED
#define PF_LOCALIZATION_PF_LOCALIZATION_H_INCLUDED

// Headers in this package
#include <pf_localization/particle_filter.h>

// Headers in ROS
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// Headers in STL
#include <memory>

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
    void twistStampedCallback(const geometry_msgs::TwistStamped::ConstPtr msg);
    void pointStampedCallback(const geometry_msgs::PointStamped::ConstPtr msg);
    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg);
    std::shared_ptr<ParticleFilter> pf_ptr_;
    int num_particles_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::string fix_position_topic_;
    std::string twist_topic_;
    std::string initial_pose_topic_;
    int update_rate_;
    ros::Publisher current_pose_pub_;
    ros::Subscriber twist_sub_;
    ros::Subscriber point_sub_;
    ros::Subscriber initial_pose_sub_;
};

#endif  //PF_LOCALIZATION_PF_LOCALIZATION_H_INCLUDED