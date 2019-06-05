#include <pf_localization/twist_data_buffer.h>

TwistDataBuffer::TwistDataBuffer(std::string key,double buffer_length) : DataBufferBase<geometry_msgs::TwistStamped>(key,buffer_length)
{

}

TwistDataBuffer::~TwistDataBuffer()
{

}

bool TwistDataBuffer::queryData(ros::Time timestamp, std::string key,geometry_msgs::TwistStamped& twist)
{
    mtx.lock();
    twist = geometry_msgs::TwistStamped();
    std::vector<geometry_msgs::TwistStamped> data = getData();
    if(data.size() == 0)
    {
        mtx.unlock();
        return false;
    }
    if(data.size() == 1)
    {
        twist = data[0];
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
        twist = interpolate(data[index],(data)[index+1],timestamp);
        mtx.unlock();
        return true;
    }
    for(int i =0; i<data.size()-1; i++)
    {
        if(data[i].header.stamp < timestamp && timestamp < data[i+1].header.stamp)
        {
            twist = interpolate(data[i],data[i+1],timestamp);
            mtx.unlock();
            return true;
        }
    }
    mtx.unlock();
    return false;
}

geometry_msgs::TwistStamped TwistDataBuffer::interpolate(geometry_msgs::TwistStamped data0,geometry_msgs::TwistStamped data1,ros::Time stamp)
{
    geometry_msgs::TwistStamped ret;
    ROS_ASSERT(data0.header.frame_id == data1.header.frame_id);
    ret.header.frame_id = data0.header.frame_id;
    ret.header.stamp = stamp;
    ret.twist.linear.x = ((data0.twist.linear.x*(data1.header.stamp-stamp).toSec()) + 
        (data1.twist.linear.x*(stamp-data0.header.stamp).toSec()))
        /(data1.header.stamp-data0.header.stamp).toSec();
    ret.twist.linear.y = ((data0.twist.linear.y*(data1.header.stamp-stamp).toSec()) + 
        (data1.twist.linear.y*(stamp-data0.header.stamp).toSec()))
        /(data1.header.stamp-data0.header.stamp).toSec();
    ret.twist.linear.z = ((data0.twist.linear.z*(data1.header.stamp-stamp).toSec()) + 
        (data1.twist.linear.z*(stamp-data0.header.stamp).toSec()))
        /(data1.header.stamp-data0.header.stamp).toSec();
    ret.twist.angular.x = ((data0.twist.angular.x*(data1.header.stamp-stamp).toSec()) + 
        (data1.twist.angular.x*(stamp-data0.header.stamp).toSec()))
        /(data1.header.stamp-data0.header.stamp).toSec();
    ret.twist.angular.y = ((data0.twist.angular.y*(data1.header.stamp-stamp).toSec()) + 
        (data1.twist.angular.y*(stamp-data0.header.stamp).toSec()))
        /(data1.header.stamp-data0.header.stamp).toSec();
    ret.twist.angular.z = ((data0.twist.angular.z*(data1.header.stamp-stamp).toSec()) + 
        (data1.twist.angular.z*(stamp-data0.header.stamp).toSec()))
        /(data1.header.stamp-data0.header.stamp).toSec();
    return ret;
}