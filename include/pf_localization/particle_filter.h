#ifndef PF_LOCALIZATION_PARTICLE_FILTER_H_INCLUDED
#define PF_LOCALIZATION_PARTICLE_FILTER_H_INCLUDED

#include <quaternion_operation/quaternion_operation.h>

//headers in ROS
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>


class ParticleFilter
{
public:
    ParticleFilter();
    ~ParticleFilter();
};

#endif  //PF_LOCALIZATION_PARTICLE_FILTER_H_INCLUDED