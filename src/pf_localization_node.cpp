// Headers in ROS
#include <ros/ros.h>

// Headers in this package
#include <pf_localization/pf_localization.h>

// Headers in GLOG
#include <glog/logging.h>

int main(int argc, char *argv[])
{
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    ros::init(argc, argv, "pf_localization_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    PfLocalization localization(nh,pnh);
    localization.run();
    ros::spin();
    return 0;
}