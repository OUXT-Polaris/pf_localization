#ifndef PF_LOCALIZATION_DATA_BUFFER_BASE_H_INCLUDED
#define PF_LOCALIZATION_DATA_BUFFER_BASE_H_INCLUDED

//headers in STL
#include <vector>
#include <map>
#include <mutex>

//headers in ROS
#include <ros/ros.h>

template<class T>
class DataBufferBase
{
public:
    DataBufferBase(std::string key,double buffer_length) : key(key),buffer_length(buffer_length){}
    ~DataBufferBase(){}
    void queryData(ros::Time from,ros::Time to,std::string key,std::vector<T>& pose)
    {
        mtx_.lock();
        std::sort(data_.begin(), data_.end(), std::bind(&DataBufferBase::compareTwistTimeStamp, this,std::placeholders::_1, std::placeholders::_2));
        mtx_.unlock();
    }
    virtual bool queryData(ros::Time timestamp, std::string key,T& pose){};
    const std::string key;
    const double buffer_length;
private:
    std::vector<T> data_;
    bool compareTimeStamp(T data0,T data1)
    {
        return data0.header.stamp < data1.header.stamp;
    }
    std::mutex mtx_;
};

#endif  //PF_LOCALIZATION_DATA_BUFFER_BASE_H_INCLUDED