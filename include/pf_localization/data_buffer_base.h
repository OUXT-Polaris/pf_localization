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
    void queryData(ros::Time from,ros::Time to,std::string key,std::vector<T>& ret)
    {
        update();
        mtx.lock();
        ret.clear();
        if(data_.size() == 0)
        {
            mtx.unlock();
            return;
        }
        for(auto itr = data_.begin(); itr != data_.end(); itr++)
        {
            if(itr->header.stamp > from && itr->header.stamp < to)
            {
                ret.push_back(*itr);
            }
        }
        mtx.unlock();
        return;
    }
    void addData(T data)
    {
        update();
        mtx.lock();
        data_.push_back(data);
        mtx.unlock();
        return;
    }
    virtual bool queryData(ros::Time timestamp, std::string key,T& ret) = 0;
    virtual T interpolate(T data0,T data1,ros::Time stamp) = 0;
    const std::string key;
    const double buffer_length;
    std::mutex mtx;
    std::vector<T> getData(){return data_;};
private:
    std::vector<T> data_;
    bool compareTimeStamp(T data0,T data1)
    {
        return data0.header.stamp < data1.header.stamp;
    }
    void reorderData()
    {
        mtx.lock();
        std::sort(data_.begin(), data_.end(), std::bind(&DataBufferBase::compareTwistTimeStamp, this,std::placeholders::_1, std::placeholders::_2));
        mtx.unlock();
    }
    void update()
    {
        std::vector<T> data;
        mtx.lock();
        ros::Time now = ros::Time::now();
        ros::Time target_timestamp = now - ros::Duration(buffer_length);
        for(auto itr = data_.begin(); itr != data_.end(); itr++)
        {
            if(itr->header.stamp > target_timestamp)
            {
                data.push_back(*itr);
            }
        }
        data_ = data;
        mtx.unlock();
    }
};

#endif  //PF_LOCALIZATION_DATA_BUFFER_BASE_H_INCLUDED