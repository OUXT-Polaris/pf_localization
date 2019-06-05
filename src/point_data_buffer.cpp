#include <pf_localization/point_data_buffer.h>

PointDataBuffer::PointDataBuffer(std::string key,double buffer_length) : DataBufferBase<geometry_msgs::PointStamped>(key,buffer_length)
{

}

PointDataBuffer::~PointDataBuffer()
{

}

bool PointDataBuffer::queryData(ros::Time timestamp, std::string key,geometry_msgs::PointStamped& point)
{
    mtx.lock();
    point = geometry_msgs::PointStamped();
    std::vector<geometry_msgs::PointStamped> data = getData();
    if(data.size() == 0)
    {
        mtx.unlock();
        return false;
    }
    if(data.size() == 1)
    {
        point = data[0];
        mtx.unlock();
        return true;
    }
    if(data[0].header.stamp > timestamp)
    {
        mtx.unlock();
        return false;
    }
    if(data[data.size()-1].header.stamp < timestamp)
    {
        int index = data.size()-2;
        point = interpolate(data[index],(data)[index+1],timestamp);
        mtx.unlock();
        return true;
    }
    for(int i =0; i<data.size()-1; i++)
    {
        if(data[i].header.stamp < timestamp && timestamp < data[i+1].header.stamp)
        {
            point = interpolate(data[i],data[i+1],timestamp);
            mtx.unlock();
            return true;
        }
    }
    mtx.unlock();
    return false;
}

geometry_msgs::PointStamped PointDataBuffer::interpolate(geometry_msgs::PointStamped data0,geometry_msgs::PointStamped data1,ros::Time stamp)
{
    geometry_msgs::PointStamped ret;
    ROS_ASSERT(data0.header.frame_id == data1.header.frame_id);
    ret.header.frame_id = data0.header.frame_id;
    ret.header.stamp = stamp;
    ret.point.x = ((data0.point.x*(data1.header.stamp-stamp).toSec()) + 
        (data1.point.x*(stamp-data0.header.stamp).toSec()))
        /(data1.header.stamp-data0.header.stamp).toSec();
    ret.point.y = ((data0.point.y*(data1.header.stamp-stamp).toSec()) + 
        (data1.point.y*(stamp-data0.header.stamp).toSec()))
        /(data1.header.stamp-data0.header.stamp).toSec();
    ret.point.z = ((data0.point.z*(data1.header.stamp-stamp).toSec()) + 
        (data1.point.z*(stamp-data0.header.stamp).toSec()))
        /(data1.header.stamp-data0.header.stamp).toSec();
    return ret;
}