#ifndef PF_LOCALIZATION_POSE_DATA_BUFFER_H_INCLUDED
#define PF_LOCALIZATION_POSE_DATA_BUFFER_H_INCLUDED

#include <pf_localization/data_buffer_base.h>
#include <geometry_msgs/PoseStamped.h>

class PoseDataBuffer : public DataBufferBase<geometry_msgs::PoseStamped>
{
public:
    PoseDataBuffer(std::string key,double buffer_length);
    ~PoseDataBuffer();
    bool queryData(ros::Time timestamp, std::string key,geometry_msgs::PoseStamped& pose) override;
};


#endif  //PF_LOCALIZATION_POSE_DATA_BUFFER_H_INCLUDED