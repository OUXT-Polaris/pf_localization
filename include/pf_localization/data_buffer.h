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
    void addData(std::string key,geometry_msgs::PoseStamped pose);
    void addData(std::string key,geometry_msgs::TwistStamped twist);
    void addData(std::string key,geometry_msgs::PointStamped point);
    bool queryData(ros::Time timestamp, std::string key,geometry_msgs::PoseStamped& pose);
    bool queryData(ros::Time timestamp, std::string key,geometry_msgs::TwistStamped& twist);
    bool queryData(ros::Time timestamp, std::string key,geometry_msgs::PointStamped& point);
private:
    std::map<std::string,std::vector<geometry_msgs::PoseStamped> > pose_buffer_;
    std::map<std::string,std::vector<geometry_msgs::TwistStamped> > twist_buffer_;
    std::map<std::string,std::vector<geometry_msgs::PointStamped> > point_buffer_;
    bool comparePoseTimeStamp(geometry_msgs::PoseStamped data0,geometry_msgs::PoseStamped data1);
    bool comparePointTimeStamp(geometry_msgs::PointStamped data0,geometry_msgs::PointStamped data1);
    bool compareTwistTimeStamp(geometry_msgs::TwistStamped data0,geometry_msgs::TwistStamped data1);
    void removeOldData();
    void reorderData();
};

#endif  //PF_LOCALIZATION_DATA_BUFFER_H_INCLUDED