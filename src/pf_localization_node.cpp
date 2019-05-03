// headers in ROS
#include <ros/ros.h>

#include <pf_localization/pf_localization.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pf_localization_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    PfLocalization localization(nh,pnh);
    ros::spin();
    return 0;
}