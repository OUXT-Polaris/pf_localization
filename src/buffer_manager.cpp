#include <pf_localization/buffer_manager.h>

namespace data_buffer
{
    BufferManager::BufferManager(double buffer_length) : buffer_length(buffer_length)
    {

    }

    BufferManager::~BufferManager()
    {

    }

    bool BufferManager::addData(std::string key,geometry_msgs::TwistStamped data)
    {
        bool key_found = false;
        for(auto buf_itr = twist_data_buf_ptrs_.begin(); buf_itr != twist_data_buf_ptrs_.end(); buf_itr++)
        {
            std::shared_ptr<TwistStampedDataBuffer> buf_ptr = *buf_itr;
            if(buf_ptr->key == key)
            {
                key_found = true;
                buf_ptr->addData(data);
            }
        }
        if(!key_found)
        {
            std::shared_ptr<TwistStampedDataBuffer> buf_ptr = std::make_shared<TwistStampedDataBuffer>(key,buffer_length);
            twist_data_buf_ptrs_.push_back(buf_ptr);
        }
        return true;
    }

    bool BufferManager::addData(std::string key,geometry_msgs::PointStamped data)
    {
        bool key_found = false;
        for(auto buf_itr = point_data_buf_ptrs_.begin(); buf_itr != point_data_buf_ptrs_.end(); buf_itr++)
        {
            std::shared_ptr<PointStampedDataBuffer> buf_ptr = *buf_itr;
            if(buf_ptr->key == key)
            {
                key_found = true;
                buf_ptr->addData(data);
            }
        }
        if(!key_found)
        {
            std::shared_ptr<PointStampedDataBuffer> buf_ptr = std::make_shared<PointStampedDataBuffer>(key,buffer_length);
            point_data_buf_ptrs_.push_back(buf_ptr);
        }
        return true;
    }

    bool BufferManager::addData(std::string key,geometry_msgs::PoseStamped data)
    {
        bool key_found = false;
        for(auto buf_itr = pose_data_buf_ptrs_.begin(); buf_itr != pose_data_buf_ptrs_.end(); buf_itr++)
        {
            std::shared_ptr<PoseStampedDataBuffer> buf_ptr = *buf_itr;
            if(buf_ptr->key == key)
            {
                key_found = true;
                buf_ptr->addData(data);
            }
        }
        if(!key_found)
        {
            std::shared_ptr<PoseStampedDataBuffer> buf_ptr = std::make_shared<PoseStampedDataBuffer>(key,buffer_length);
            pose_data_buf_ptrs_.push_back(buf_ptr);
        }
        return true;
    }

    bool BufferManager::queryData(ros::Time stamp,std::string key,geometry_msgs::TwistStamped data)
    {
        for(auto buf_itr = twist_data_buf_ptrs_.begin(); buf_itr != twist_data_buf_ptrs_.end(); buf_itr++)
        {
            std::shared_ptr<TwistStampedDataBuffer> buf_ptr = *buf_itr;
            if(buf_ptr->key == key)
            {
                bool result = buf_ptr->queryData(stamp,key,data);
                if(result)
                {
                    return true;
                }
                else
                {
                    ROS_ERROR_STREAM("failed to query data from twist data buffer, query key : " << key << ",timestamp : " << stamp);
                    return false;
                }
            }
        }
        return false;
    }

    bool BufferManager::queryData(ros::Time stamp,std::string key,geometry_msgs::PointStamped data)
    {
        for(auto buf_itr = point_data_buf_ptrs_.begin(); buf_itr != point_data_buf_ptrs_.end(); buf_itr++)
        {
            std::shared_ptr<PointStampedDataBuffer> buf_ptr = *buf_itr;
            if(buf_ptr->key == key)
            {
                bool result = buf_ptr->queryData(stamp,key,data);
                if(result)
                {
                    return true;
                }
                else
                {
                    ROS_ERROR_STREAM("failed to query data from point data buffer, query key : " << key << ",timestamp : " << stamp);
                    return false;
                }
            }
        }
        return false;
    }

    bool BufferManager::queryData(ros::Time stamp,std::string key,geometry_msgs::PoseStamped data)
    {
        for(auto buf_itr = pose_data_buf_ptrs_.begin(); buf_itr != pose_data_buf_ptrs_.end(); buf_itr++)
        {
            std::shared_ptr<PoseStampedDataBuffer> buf_ptr = *buf_itr;
            if(buf_ptr->key == key)
            {
                bool result = buf_ptr->queryData(stamp,key,data);
                if(result)
                {
                    return true;
                }
                else
                {
                    ROS_ERROR_STREAM("failed to query data from pose data buffer, query key : " << key << ",timestamp : " << stamp);
                    return false;
                }
            }
        }
        return false;
    }
}