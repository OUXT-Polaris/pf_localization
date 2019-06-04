#include <pf_localization/data_buffer.h>

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
    mtx_.lock();
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
    mtx_.unlock();
}

void DataBuffer::queryData(ros::Time from,ros::Time to,std::string key,std::vector<geometry_msgs::PoseStamped>& pose)
{
    reorderData();
    pose.clear();
    mtx_.lock();
    if(pose_buffer_.find(key) == pose_buffer_.end())
    {
        mtx_.unlock();
        return;
    }
    if(pose_buffer_[key].size() == 0)
    {
        mtx_.unlock();
        return;
    }
    for(auto itr = pose_buffer_[key].begin(); itr != pose_buffer_[key].end(); itr++)
    {
        if(itr->header.stamp > from && itr->header.stamp < to)
        {
            pose.push_back(*itr);
        }
    }
    mtx_.unlock();
    return;
}

void DataBuffer::queryData(ros::Time from,ros::Time to,std::string key,std::vector<geometry_msgs::TwistStamped>& twist)
{
    reorderData();
    twist.clear();
    mtx_.lock();
    if(twist_buffer_.find(key) == twist_buffer_.end())
    {
        mtx_.unlock();
        return;
    }
    if(twist_buffer_[key].size() == 0)
    {
        mtx_.unlock();
        return;
    }
    for(auto itr = twist_buffer_[key].begin(); itr != twist_buffer_[key].end(); itr++)
    {
        if(itr->header.stamp > from && itr->header.stamp < to)
        {
            twist.push_back(*itr);
        }
    }
    mtx_.unlock();
    return;
}

void DataBuffer::queryData(ros::Time from,ros::Time to,std::string key,std::vector<geometry_msgs::PointStamped>& point)
{
    reorderData();
    point.clear();
    mtx_.lock();
    if(point_buffer_.find(key) == point_buffer_.end())
    {
        mtx_.unlock();
        return;
    }
    if(point_buffer_[key].size() == 0)
    {
        mtx_.unlock();
        return;
    }
    for(auto itr = point_buffer_[key].begin(); itr != point_buffer_[key].end(); itr++)
    {
        if(itr->header.stamp > from && itr->header.stamp < to)
        {
            point.push_back(*itr);
        }
    }
    mtx_.unlock();
    return;
}

bool DataBuffer::queryData(ros::Time timestamp, std::string key,geometry_msgs::PoseStamped& pose)
{
    reorderData();
    mtx_.lock();
    pose = geometry_msgs::PoseStamped();
    if(pose_buffer_.find(key) == pose_buffer_.end())
    {
        mtx_.unlock();
        return false;
    }
    if(pose_buffer_[key].size() == 0)
    {
        mtx_.unlock();
        return false;
    }
    if(pose_buffer_[key].size() == 1)
    {
        pose = (pose_buffer_[key])[0];
        mtx_.unlock();
        return false;
    }
    if(pose_buffer_[key].begin()->header.stamp > timestamp)
    {
        mtx_.unlock();
        return false;
    }
    if(pose_buffer_[key].end()->header.stamp < timestamp)
    {
        int index = pose_buffer_[key].size()-2;
        pose = interpolate((pose_buffer_[key])[index],(pose_buffer_[key])[index+1],timestamp);
        mtx_.unlock();
        return true;
    }
    for(auto itr = pose_buffer_[key].begin(); itr != pose_buffer_[key].end(); itr++)
    {
        if(itr->header.stamp < timestamp && timestamp < (itr+1)->header.stamp)
        {
            pose = interpolate(*itr,*(itr+1),timestamp);
            mtx_.unlock();
            return true;
        }
    }
    mtx_.unlock();
    return false;
}

bool DataBuffer::queryData(ros::Time timestamp, std::string key,geometry_msgs::TwistStamped& twist)
{
    reorderData();
    mtx_.lock();
    twist = geometry_msgs::TwistStamped();
    if(twist_buffer_.find(key) == twist_buffer_.end())
    {
        mtx_.unlock();
        return false;
    }
    if(twist_buffer_[key].size() == 0)
    {
        mtx_.unlock();
        return false;
    }
    if(twist_buffer_[key].size() == 1)
    {
        twist = (twist_buffer_[key])[0];
        mtx_.unlock();
        return true;
    }
    if(twist_buffer_[key].begin()->header.stamp > timestamp)
    {
        mtx_.unlock();
        return false;
    }
    if(twist_buffer_[key].end()->header.stamp < timestamp)
    {
        int index = twist_buffer_[key].size()-2;
        twist = interpolate((twist_buffer_[key])[index],(twist_buffer_[key])[index+1],timestamp);
        mtx_.unlock();
        return true;
    }
    for(auto itr = twist_buffer_[key].begin(); itr != (twist_buffer_[key].end()-1); itr++)
    {
        if(itr->header.stamp < timestamp && timestamp < (itr+1)->header.stamp)
        {
            twist = interpolate(*itr,*(itr+1),timestamp);
            mtx_.unlock();
            return true;
        }
    }
    mtx_.unlock();
    return false;
}

bool DataBuffer::queryData(ros::Time timestamp, std::string key,geometry_msgs::PointStamped& point)
{
    reorderData();
    mtx_.lock();
    point = geometry_msgs::PointStamped();
    if(point_buffer_.find(key) == point_buffer_.end())
    {
        mtx_.unlock();
        return false;
    }
    if(point_buffer_[key].size() == 0)
    {
        mtx_.unlock();
        return false;
    }
    if(point_buffer_[key].size() == 1)
    {
        point = (point_buffer_[key])[0];
        mtx_.unlock();
        return true;
    }
    if(point_buffer_[key].begin()->header.stamp > timestamp)
    {
        mtx_.unlock();
        return false;
    }
    if(point_buffer_[key].end()->header.stamp < timestamp)
    {
        int index = point_buffer_[key].size()-2;
        point = interpolate((point_buffer_[key])[index],(point_buffer_[key])[index+1],timestamp);
        mtx_.unlock();
        return true;
    }
    for(auto itr = point_buffer_[key].begin(); itr != (point_buffer_[key].end()-1); itr++)
    {
        if(itr->header.stamp < timestamp && timestamp < (itr+1)->header.stamp)
        {
            point = interpolate(*itr,*(itr+1),timestamp);
            mtx_.unlock();
            return true;
        }
    }
    return false;
}

void DataBuffer::addData(std::string key,geometry_msgs::PoseStamped pose)
{
    mtx_.lock();
    pose_buffer_[key].push_back(pose);
    mtx_.unlock();
    removeOldData();
    reorderData();
    return;
}

void DataBuffer::addData(std::string key,geometry_msgs::PointStamped point)
{
    mtx_.lock();
    point_buffer_[key].push_back(point);
    mtx_.unlock();
    removeOldData();
    reorderData();
    return;
}

void DataBuffer::addData(std::string key,geometry_msgs::TwistStamped twist)
{
    mtx_.lock();
    twist_buffer_[key].push_back(twist);
    mtx_.unlock();
    removeOldData();
    reorderData();
    return;
}

void DataBuffer::removeOldData()
{
    mtx_.lock();
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
    mtx_.unlock();
}

geometry_msgs::PoseStamped DataBuffer::interpolate(geometry_msgs::PoseStamped data0,geometry_msgs::PoseStamped data1,ros::Time stamp)
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

geometry_msgs::TwistStamped DataBuffer::interpolate(geometry_msgs::TwistStamped data0,geometry_msgs::TwistStamped data1,ros::Time stamp)
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

geometry_msgs::PointStamped DataBuffer::interpolate(geometry_msgs::PointStamped data0,geometry_msgs::PointStamped data1,ros::Time stamp)
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