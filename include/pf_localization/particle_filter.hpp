// Copyright (c) 2019 OUXT Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PF_LOCALIZATION__PARTICLE_FILTER_HPP_
#define PF_LOCALIZATION__PARTICLE_FILTER_HPP_

// headers in this package
#include <pf_localization/twist_estimator.hpp>

// headers in ROS
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.h>
#include <geometry_msgs_data_buffer/pose_stamped_data_buffer.h>
#include <geometry_msgs_data_buffer/twist_stamped_data_buffer.h>
#include <quaternion_operation/quaternion_operation.h>

// headers in Boost
#include <boost/optional.hpp>

// headers in STL
#include <float.h>
#include <map>
#include <random>
#include <memory>
#include <string>
#include <vector>

namespace pf_localization
{
struct Particle
{
  geometry_msgs::msg::PoseStamped pose;
  double weight;
};

class ParticleFilter
{
public:
  ParticleFilter(
    int num_particles, double buffer_length, bool estimate_3d_pose,
    double expansion_reset_ess_threashold, double max_expansion_orientation,
    double max_expantion_position,
    double sensor_reset_ess_threashold, double max_sensor_reset_orientation,
    double max_sensor_reset_position, double sensor_reset_radius,
    double weight_position, double weight_orientation, rclcpp::Clock::SharedPtr clock);
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
  void updateTwist(geometry_msgs::msg::TwistStamped twist);
  void updatePose(geometry_msgs::msg::PoseStamped pose);
  boost::optional<geometry_msgs::msg::PoseStamped> estimateCurrentPose(rclcpp::Time stamp);
  boost::optional<geometry_msgs::msg::TwistStamped> getCurrentTwist()
  {
    return twist_estimator_->estimateTwist();
  }
  void setInitialPose(geometry_msgs::msg::PoseStamped pose);
  boost::optional<geometry_msgs::msg::PoseStamped> getInitialPose();
  std::vector<Particle> getParticles() {return particles_;}
  double getEffectiveSampleSize();

private:
  double getTotalWeights();
  void normalizeWeights();
  void expansionReset();
  void sensorReset(geometry_msgs::msg::PoseStamped pose);
  bool checkQuaternion(geometry_msgs::msg::Quaternion quat);
  void resampling();
  std::map<std::string, double> twist_weights_;
  std::map<std::string, double> point_weights_;
  std::vector<Particle> particles_;
  boost::optional<geometry_msgs::msg::PoseStamped> current_pose_;
  boost::optional<geometry_msgs::msg::PoseStamped> initial_pose_;
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
}  // namespace pf_localization

#endif  // PF_LOCALIZATION__PARTICLE_FILTER_HPP_
