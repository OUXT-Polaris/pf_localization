#ifndef PF_LOCALIZATION_PARTICLE_FILTER_H_INCLUDED
#define PF_LOCALIZATION_PARTICLE_FILTER_H_INCLUDED

//headers in ROS
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs_data_buffer/pose_stamped_data_buffer.h>
#include <geometry_msgs_data_buffer/twist_stamped_data_buffer.h>
#include <quaternion_operation/quaternion_operation.h>

//headers in Boost
#include <boost/optional.hpp>

//headers in STL
#include <float.h>
#include <map>
#include <random>

struct Particle
{
    geometry_msgs::PoseStamped pose;
    double weight;
};

class ParticleFilter
{
public:
    ParticleFilter(int num_particles,double buffer_length,bool estimate_3d_pose);
    ~ParticleFilter();
    const int num_particles;
    const double buffer_length;
    const bool estimate_3d_pose;
    void updateTwist(geometry_msgs::TwistStamped twist);
    void updatePose(geometry_msgs::PoseStamped pose);
    boost::optional<geometry_msgs::PoseStamped> estimateCurrentPose(ros::Time stamp);
    void setInitialPose(geometry_msgs::PoseStamped pose);
    boost::optional<geometry_msgs::PoseStamped> getInitialPose();
    std::vector<Particle> getParticles(){return particles_;};
    void reset(geometry_msgs::PoseStamped pose);
private:
    bool checkQuaternion(geometry_msgs::Quaternion quat);
    //data_buffer::BufferManager buffer_manager_;
    std::map<std::string,double> twist_weights_;
    std::map<std::string,double> point_weights_;
    boost::optional<geometry_msgs::TwistStamped> estimateTwist(ros::Time stamp);
    boost::optional<geometry_msgs::PoseStamped> estimatePose(ros::Time stamp);
    std::vector<Particle> particles_;
    boost::optional<geometry_msgs::PoseStamped> current_pose_;
    boost::optional<geometry_msgs::PoseStamped> initial_pose_;
    std::random_device seed_gen_;
    std::default_random_engine engine_;
    std::normal_distribution<> position_dist_;
    std::normal_distribution<> rotation_dist_;
    std::mt19937 mt_;
    std::uniform_real_distribution<double> uniform_dist_;
    data_buffer::PoseStampedDataBuffer pose_buf_;
    data_buffer::TwistStampedDataBuffer twist_buf_;
};

#endif  //PF_LOCALIZATION_PARTICLE_FILTER_H_INCLUDED