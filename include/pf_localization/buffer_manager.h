#ifndef PF_LOCALIZATION__MANAGER_H_INCLUDED
#define PF_LOCALIZATION_BUFFER_MANAGER_H_INCLUDED

// headers in this package
#include <geometry_msgs_data_buffer/twist_stamped_data_buffer.h>
#include <geometry_msgs_data_buffer/point_stamped_data_buffer.h>
#include <geometry_msgs_data_buffer/pose_stamped_data_buffer.h>

// headers in stl
#include <vector>
#include <memory>

namespace data_buffer
{
    class BufferManager
    {
    public:
        BufferManager(double buffer_length);
        ~BufferManager();
        bool addData(std::string key,geometry_msgs::TwistStamped data);
        bool addData(std::string key,geometry_msgs::PointStamped data);
        bool addData(std::string key,geometry_msgs::PoseStamped data);
        bool queryData(ros::Time stamp,std::string key,geometry_msgs::TwistStamped data);
        bool queryData(ros::Time stamp,std::string key,geometry_msgs::PointStamped data);
        bool queryData(ros::Time stamp,std::string key,geometry_msgs::PoseStamped data);
        const double buffer_length;
    private:
        std::vector<std::shared_ptr<TwistStampedDataBuffer> > twist_data_buf_ptrs_;
        std::vector<std::shared_ptr<PointStampedDataBuffer> > point_data_buf_ptrs_;
        std::vector<std::shared_ptr<PoseStampedDataBuffer> > pose_data_buf_ptrs_;
    };
}

#endif  //PF_LOCALIZATION_BUFFER_MANAGER_H_INCLUDED