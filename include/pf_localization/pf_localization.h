#ifndef PF_LOCALIZATION_PF_LOCALIZATION_H_INCLUDED
#define PF_LOCALIZATION_PF_LOCALIZATION_H_INCLUDED

// Headers in this package
#include <pf_localization/particle_filter.h>

// Headers in ROS
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

// Headers in STL
#include <memory>

class PfLocalization
{
public:
    PfLocalization(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~PfLocalization();
private:
    void twistStampedCallback(const geometry_msgs::TwistStamped::ConstPtr msg);
    void pointStampedCallback(const geometry_msgs::PointStamped::ConstPtr msg);
    std::shared_ptr<ParticleFilter> pf_ptr_;
};

#endif  //PF_LOCALIZATION_PF_LOCALIZATION_H_INCLUDED