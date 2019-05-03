#include <pf_localization/data_buffer.h>

// Headers in ROS
#include <quaternion_operation/quaternion_operation.h>

DataBuffer::DataBuffer(double buffer_length) : buffer_length(buffer_length)
{

}

DataBuffer::~DataBuffer()
{

}

bool DataBuffer::comparePoseTimeStamp(geometry_msgs::PoseStamped data0,geometry_msgs::PoseStamped data1)
{
    return data0.header.stamp < data1.header.stamp;
}

bool DataBuffer::comparePointTimeStamp(geometry_msgs::PointStamped data0,geometry_msgs::PointStamped data1)
{
    return data0.header.stamp < data1.header.stamp;
}

bool DataBuffer::compareTwistTimeStamp(geometry_msgs::TwistStamped data0,geometry_msgs::TwistStamped data1)
{
    return data0.header.stamp < data1.header.stamp;
}

void DataBuffer::reorderData()
{
    for(auto key_itr = pose_buffer_.begin(); key_itr != pose_buffer_.end(); key_itr++)
    {
        std::sort(pose_buffer_[key_itr->first].begin(), pose_buffer_[key_itr->first].end(), 
            std::bind(&DataBuffer::comparePoseTimeStamp, this,std::placeholders::_1, std::placeholders::_2));
    }
    for(auto key_itr = point_buffer_.begin(); key_itr != point_buffer_.end(); key_itr++)
    {
        std::sort(point_buffer_[key_itr->first].begin(), point_buffer_[key_itr->first].end(), 
            std::bind(&DataBuffer::comparePointTimeStamp, this,std::placeholders::_1, std::placeholders::_2));
    }
    for(auto key_itr = twist_buffer_.begin(); key_itr != twist_buffer_.end(); key_itr++)
    {
        std::sort(twist_buffer_[key_itr->first].begin(), twist_buffer_[key_itr->first].end(), 
            std::bind(&DataBuffer::compareTwistTimeStamp, this,std::placeholders::_1, std::placeholders::_2));
    }
}

void DataBuffer::addData(std::string key,geometry_msgs::PoseStamped pose)
{
    pose_buffer_[key].push_back(pose);
    removeOldData();
    reorderData();
    return;
}

void DataBuffer::addData(std::string key,geometry_msgs::PointStamped point)
{
    point_buffer_[key].push_back(point);
    removeOldData();
    reorderData();
    return;
}

void DataBuffer::addData(std::string key,geometry_msgs::TwistStamped twist)
{
    twist_buffer_[key].push_back(twist);
    removeOldData();
    reorderData();
    return;
}

void DataBuffer::removeOldData()
{
    ros::Time now = ros::Time::now();
    ros::Time target_timestamp = now - ros::Duration(buffer_length);
    for(auto key_itr = pose_buffer_.begin(); key_itr != pose_buffer_.end(); key_itr++)
    {
        std::vector<geometry_msgs::PoseStamped> data;
        for(auto itr = pose_buffer_[key_itr->first].begin(); itr != pose_buffer_[key_itr->first].end(); itr++)
        {
            if(itr->header.stamp > target_timestamp)
            {
                data.push_back(*itr);
            }
        }
        pose_buffer_[key_itr->first] = data;
    }
    for(auto key_itr = twist_buffer_.begin(); key_itr != twist_buffer_.end(); key_itr++)
    {
        std::vector<geometry_msgs::TwistStamped> data;
        for(auto itr = twist_buffer_[key_itr->first].begin(); itr != twist_buffer_[key_itr->first].end(); itr++)
        {
            if(itr->header.stamp > target_timestamp)
            {
                data.push_back(*itr);
            }
        }
        twist_buffer_[key_itr->first] = data;
    }
    for(auto key_itr = point_buffer_.begin(); key_itr != point_buffer_.end(); key_itr++)
    {
        std::vector<geometry_msgs::PointStamped> data;
        for(auto itr = point_buffer_[key_itr->first].begin(); itr != point_buffer_[key_itr->first].end(); itr++)
        {
            if(itr->header.stamp > target_timestamp)
            {
                data.push_back(*itr);
            }
        }
        point_buffer_[key_itr->first] = data;
    }
}