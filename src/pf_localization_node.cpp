// headers in ROS
#include <ros/ros.h>


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pf_localization_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ros::spin();
    return 0;
}