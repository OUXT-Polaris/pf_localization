#ifndef PF_LOCALIZATION_TWIST_DATA_BUFFER_H_INCLUDED
#define PF_LOCALIZATION_TWIST_DATA_BUFFER_H_INCLUDED

//headers in this package
#include <pf_localization/data_buffer_base.h>

//headers in ROS
#include <geometry_msgs/TwistStamped.h>
#include <quaternion_operation/quaternion_operation.h>

class TwistDataBuffer : public DataBufferBase<geometry_msgs::TwistStamped>
{
public:
    TwistDataBuffer(std::string key,double buffer_length);
    ~TwistDataBuffer();
    bool queryData(ros::Time timestamp, std::string key,geometry_msgs::TwistStamped& Twist) override;
private:
    geometry_msgs::TwistStamped interpolate(geometry_msgs::TwistStamped data0,geometry_msgs::TwistStamped data1,ros::Time stamp) override;
};


#endif  //PF_LOCALIZATION_TWIST_DATA_BUFFER_H_INCLUDED