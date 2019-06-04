#include <pf_localization/pose_data_buffer.h>

PoseDataBuffer::PoseDataBuffer(std::string key,double buffer_length) : DataBufferBase<geometry_msgs::PoseStamped>(key,buffer_length)
{

}

PoseDataBuffer::~PoseDataBuffer()
{

}

bool PoseDataBuffer::queryData(ros::Time timestamp, std::string key,geometry_msgs::PoseStamped& pose)
{
    mtx.lock();
    pose = geometry_msgs::PoseStamped();
    std::vector<geometry_msgs::PoseStamped> data = getData();
    if(data.size() == 0)
    {
        mtx.unlock();
        return false;
    }
    if(data.size() == 1)
    {
        pose = data[0];
        mtx.unlock();
        return true;
    }
    if(data.begin()->header.stamp > timestamp)
    {
        mtx.unlock();
        return false;
    }
    if(data.end()->header.stamp < timestamp)
    {
        int index = data.size()-2;
        pose = interpolate(data[index],(data)[index+1],timestamp);
        mtx.unlock();
        return true;
    }
    for(auto itr = data.begin(); itr != (data.end()-1); itr++)
    {
        if(itr->header.stamp < timestamp && timestamp < (itr+1)->header.stamp)
        {
            pose = interpolate(*itr,*(itr+1),timestamp);
            mtx.unlock();
            return true;
        }
    }
    mtx.unlock();
    return false;
}

geometry_msgs::PoseStamped PoseDataBuffer::interpolate(geometry_msgs::PoseStamped data0,geometry_msgs::PoseStamped data1,ros::Time stamp)
{
    geometry_msgs::PoseStamped ret;
    ROS_ASSERT(data0.header.frame_id == data1.header.frame_id);
    ret.header.frame_id = data0.header.frame_id;
    ret.header.stamp = stamp;
    ret.pose.position.x = ((data0.pose.position.x*(data1.header.stamp-stamp).toSec()) + 
        (data1.pose.position.x*(stamp-data0.header.stamp).toSec()))
        /(data1.header.stamp-data0.header.stamp).toSec();
    ret.pose.position.y = ((data0.pose.position.y*(data1.header.stamp-stamp).toSec()) + 
        (data1.pose.position.y*(stamp-data0.header.stamp).toSec()))
        /(data1.header.stamp-data0.header.stamp).toSec();
    ret.pose.position.z = ((data0.pose.position.z*(data1.header.stamp-stamp).toSec()) + 
        (data1.pose.position.z*(stamp-data0.header.stamp).toSec()))
        /(data1.header.stamp-data0.header.stamp).toSec();
    double ratio = (stamp-data0.header.stamp).toSec()/(data1.header.stamp-data0.header.stamp).toSec();
    ret.pose.orientation = quaternion_operation::slerp(data0.pose.orientation,data1.pose.orientation,ratio);
    return ret;
}