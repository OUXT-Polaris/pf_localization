#ifndef PF_LOCALIZATION__MANAGER_H_INCLUDED
#define PF_LOCALIZATION_BUFFER_MANAGER_H_INCLUDED

// headers in this package
#include <pf_localization/twist_data_buffer.h>
#include <pf_localization/point_data_buffer.h>
#include <pf_localization/pose_data_buffer.h>

// headers in stl
#include <vector>
#include <memory>

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
    std::vector<std::shared_ptr<TwistDataBuffer> > twist_data_buf_ptrs_;
    std::vector<std::shared_ptr<PointDataBuffer> > point_data_buf_ptrs_;
    std::vector<std::shared_ptr<PoseDataBuffer> > pose_data_buf_ptrs_;
};

#endif  //PF_LOCALIZATION_BUFFER_MANAGER_H_INCLUDED