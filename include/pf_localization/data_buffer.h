#ifndef PF_LOCALIZATION_DATA_BUFFER_H_INCLUDED
#define PF_LOCALIZATION_DATA_BUFFER_H_INCLUDED

//headers in STL
#include <vector>
#include <map>

//headers in ROS
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

class DataBuffer
{
public:
    DataBuffer(double buffer_length);
    ~DataBuffer();
    const double buffer_length;
private:
    std::map<std::string,std::vector<geometry_msgs::PoseStamped> > pose_buffer_;
    std::map<std::string,std::vector<geometry_msgs::TwistStamped> > twist_buffer_;
    std::map<std::string,std::vector<geometry_msgs::PointStamped> > point_buffer_;
    void removeOldData();
};

#endif  //PF_LOCALIZATION_DATA_BUFFER_H_INCLUDED