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

#ifndef PF_LOCALIZATION__TWIST_ESTIMATOR_HPP_
#define PF_LOCALIZATION__TWIST_ESTIMATOR_HPP_

// Headers in ROS
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <quaternion_operation/quaternion_operation.h>

// Headers in Boost
#include <boost/circular_buffer.hpp>
#include <boost/optional.hpp>

// Headers in STL
#include <string>

namespace pf_localization
{
class TwistEstimator
{
public:
  explicit TwistEstimator(std::string robot_frame_id);
  ~TwistEstimator();
  void add(geometry_msgs::msg::PoseStamped pose);
  void clear();
  boost::optional<geometry_msgs::msg::TwistStamped> estimateTwist();
  const std::string robot_frame_id;

private:
  boost::circular_buffer<geometry_msgs::msg::PoseStamped> pose_buffer_;
};
}  // namespace pf_localization

#endif  // PF_LOCALIZATION__TWIST_ESTIMATOR_HPP_
