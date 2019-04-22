#include <pf_localization/data_buffer.h>

DataBuffer::DataBuffer(double buffer_length) : buffer_length(buffer_length)
{

}

DataBuffer::~DataBuffer()
{

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