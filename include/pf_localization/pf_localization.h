#ifndef PF_LOCALIZATION_PF_LOCALIZATION_H_INCLUDED
#define PF_LOCALIZATION_PF_LOCALIZATION_H_INCLUDED

// Headers in ROS
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

class PfLocalization
{
public:
    PfLocalization(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~PfLocalization();
private:
    void twistStampedCallback(const geometry_msgs::TwistStamped::ConstPtr msg);
};

#endif  //PF_LOCALIZATION_PF_LOCALIZATION_H_INCLUDED