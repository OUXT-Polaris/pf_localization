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

// Headers in this package
#include <pf_localization/twist_estimator.h>

struct Particle
{
    geometry_msgs::PoseStamped pose;
    double weight;
};

class ParticleFilter
{
public:
    ParticleFilter(int num_particles,double buffer_length,bool estimate_3d_pose,std::string robot_frame_id,
        double expansion_reset_ess_threashold,double max_expansion_orientation,double max_expantion_position,
        double sensor_reset_ess_threashold,double max_sensor_reset_orientation,double max_sensor_reset_position,double sensor_reset_radius,
        double weight_position,double weight_orientation);
    ~ParticleFilter();
    const int num_particles;
    const double buffer_length;
    const bool estimate_3d_pose;
    const double expansion_reset_ess_threashold;
    const double max_expansion_orientation;
    const double max_expantion_position;
    const double sensor_reset_ess_threashold;
    const double max_sensor_reset_orientation;
    const double max_sensor_reset_position;
    const double sensor_reset_radius;
    const double weight_position;
    const double weight_orientation;
    void updateTwist(geometry_msgs::TwistStamped twist);
    void updatePose(geometry_msgs::PoseStamped pose);
    boost::optional<geometry_msgs::PoseStamped> estimateCurrentPose(ros::Time stamp);
    boost::optional<geometry_msgs::TwistStamped> getCurrentTwist()
    {
        return twist_estimator_->estimateTwist();
    }
    void setInitialPose(geometry_msgs::PoseStamped pose);
    boost::optional<geometry_msgs::PoseStamped> getInitialPose();
    std::vector<Particle> getParticles(){return particles_;};
    double getEffectiveSampleSize();
private:
    void normalizeWeights();
    void expansionReset();
    void sensorReset(geometry_msgs::PoseStamped pose);
    bool checkQuaternion(geometry_msgs::Quaternion quat);
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
    std::unique_ptr<TwistEstimator> twist_estimator_;
};

#endif  //PF_LOCALIZATION_PARTICLE_FILTER_H_INCLUDED