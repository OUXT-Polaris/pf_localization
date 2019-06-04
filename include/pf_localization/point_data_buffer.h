#ifndef PF_LOCALIZATION_POINT_DATA_BUFFER_H_INCLUDED
#define PF_LOCALIZATION_POINT_DATA_BUFFER_H_INCLUDED

//headers in this package
#include <pf_localization/data_buffer_base.h>

//headers in ROS
#include <geometry_msgs/PointStamped.h>

class PointDataBuffer : public DataBufferBase<geometry_msgs::PointStamped>
{
public:
    PointDataBuffer(std::string key,double buffer_length);
    ~PointDataBuffer();
    bool queryData(ros::Time timestamp, std::string key,geometry_msgs::PointStamped& pose) override;
private:
    geometry_msgs::PointStamped interpolate(geometry_msgs::PointStamped data0,geometry_msgs::PointStamped data1,ros::Time stamp) override;
};

#endif  //PF_LOCALIZATION_POSE_DATA_BUFFER_H_INCLUDED