#ifndef PF_LOCALIZATION_POSE_DATA_BUFFER_H_INCLUDED
#define PF_LOCALIZATION_POSE_DATA_BUFFER_H_INCLUDED

//headers in this package
#include <pf_localization/data_buffer_base.h>

//headers in ROS
#include <geometry_msgs/PoseStamped.h>
#include <quaternion_operation/quaternion_operation.h>

class PoseDataBuffer : public DataBufferBase<geometry_msgs::PoseStamped>
{
public:
    PoseDataBuffer(std::string key,double buffer_length);
    ~PoseDataBuffer();
    bool queryData(ros::Time timestamp, std::string key,geometry_msgs::PoseStamped& pose) override;
private:
    geometry_msgs::PoseStamped interpolate(geometry_msgs::PoseStamped data0,geometry_msgs::PoseStamped data1,ros::Time stamp) override;
};


#endif  //PF_LOCALIZATION_POSE_DATA_BUFFER_H_INCLUDED