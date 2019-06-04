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
    void addPointData(std::string key,geometry_msgs::PointStamped data);
    const double buffer_length;
private:
    std::vector<TwistDataBuffer> twist_data_bufs_;
    std::vector<std::shared_ptr<PointDataBuffer> > point_data_bufs_;
    std::vector<PoseDataBuffer> pose_data_bufs_;
};

#endif  //PF_LOCALIZATION_BUFFER_MANAGER_H_INCLUDED