#include <pf_localization/buffer_manager.h>

BufferManager::BufferManager(double buffer_length) : buffer_length(buffer_length)
{

}

BufferManager::~BufferManager()
{

}

void BufferManager::addPointData(std::string key,geometry_msgs::PointStamped data)
{
    bool key_found = false;
    for(auto buf_itr = point_data_bufs_.begin(); buf_itr != point_data_bufs_.end(); buf_itr++)
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
        point_data_bufs_.push_back(buf_ptr);
    }
}