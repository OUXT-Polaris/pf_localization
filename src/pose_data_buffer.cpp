#include <pf_localization/pose_data_buffer.h>

PoseDataBuffer::PoseDataBuffer(std::string key,double buffer_length) : DataBufferBase<geometry_msgs::PoseStamped>(key,buffer_length)
{

}

PoseDataBuffer::~PoseDataBuffer()
{

}

bool PoseDataBuffer::queryData(ros::Time timestamp, std::string key,geometry_msgs::PoseStamped& pose)
{
    return true;
}