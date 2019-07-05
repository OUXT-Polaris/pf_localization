#ifndef PF_LOCALIZATION_TWIST_ESTIMATOR_H_INCLUDED
#define PF_LOCALIZATION_TWIST_ESTIMATOR_H_INCLUDED

// Headers in ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <quaternion_operation/quaternion_operation.h>

// Headers in Boost
#include <boost/circular_buffer.hpp>
#include <boost/optional.hpp>

class TwistEstimator
{
public:
    TwistEstimator(std::string robot_frame_id);
    ~TwistEstimator();
    void add(geometry_msgs::PoseStamped pose);
    void clear();
    boost::optional<geometry_msgs::TwistStamped> estimateTwist();
    const std::string robot_frame_id;
private:
    boost::circular_buffer<geometry_msgs::PoseStamped> pose_buffer_;
};

#endif  //PF_LOCALIZATION_TWIST_ESTIMATOR_H_INCLUDED