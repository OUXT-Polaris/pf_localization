#include <pf_localization/buffer_manager.h>

BufferManager::BufferManager(double buffer_length) : buffer_length(buffer_length)
{

}

BufferManager::~BufferManager()
{

}

template <typename T>
bool BufferManager::addData(std::string key,T data)
{
    if(std::is_same<T, geometry_msgs::PointStamped>::value)
    {
        bool key_found = false;
        for(auto buf_itr = point_data_buf_ptrs_.begin(); buf_itr != point_data_buf_ptrs_.end(); buf_itr++)
        {
            std::shared_ptr<PointDataBuffer> buf_ptr = *buf_itr;
            if(buf_ptr->key == key)
            {
                key_found = true;
                buf_ptr->addData(data);
            }
        }
        if(!key_found)
        {
            std::shared_ptr<PointDataBuffer> buf_ptr = std::make_shared<PointDataBuffer>(key,buffer_length);
            point_data_buf_ptrs_.push_back(buf_ptr);
        }
        return true;
    }
    else if(std::is_same<T, geometry_msgs::PoseStamped>::value)
    {
        bool key_found = false;
        for(auto buf_itr = pose_data_buf_ptrs_.begin(); buf_itr != pose_data_buf_ptrs_.end(); buf_itr++)
        {
            std::shared_ptr<PoseDataBuffer> buf_ptr = *buf_itr;
            if(buf_ptr->key == key)
            {
                key_found = true;
                buf_ptr->addData(data);
            }
        }
        if(!key_found)
        {
            std::shared_ptr<PoseDataBuffer> buf_ptr = std::make_shared<PoseDataBuffer>(key,buffer_length);
            pose_data_buf_ptrs_.push_back(buf_ptr);
        }
        return true;
    }
    else if(std::is_same<T, geometry_msgs::TwistStamped>::value)
    {
        bool key_found = false;
        for(auto buf_itr = twist_data_buf_ptrs_.begin(); buf_itr != twist_data_buf_ptrs_.end(); buf_itr++)
        {
            std::shared_ptr<TwistDataBuffer> buf_ptr = *buf_itr;
            if(buf_ptr->key == key)
            {
                key_found = true;
                buf_ptr->addData(data);
            }
        }
        if(!key_found)
        {
            std::shared_ptr<TwistDataBuffer> buf_ptr = std::make_shared<TwistDataBuffer>(key,buffer_length);
            twist_data_buf_ptrs_.push_back(buf_ptr);
        }
        return true;
    }
    return false;
}