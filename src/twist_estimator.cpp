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

// headers in this package
#include <pf_localization/twist_estimator.hpp>

// headers in STL
#include <string>

namespace pf_localization
{
TwistEstimator::TwistEstimator(std::string robot_frame_id)
: robot_frame_id(robot_frame_id)
{
  pose_buffer_ = boost::circular_buffer<geometry_msgs::msg::PoseStamped>(2);
}

TwistEstimator::~TwistEstimator()
{
}

void TwistEstimator::add(geometry_msgs::msg::PoseStamped pose)
{
  pose_buffer_.push_back(pose);
}

void TwistEstimator::clear()
{
  pose_buffer_.clear();
}

boost::optional<geometry_msgs::msg::TwistStamped> TwistEstimator::estimateTwist()
{
  if (pose_buffer_.size() != 2) {
    return boost::none;
  }
  rclcpp::Time t0_stamp = pose_buffer_[0].header.stamp;
  rclcpp::Time t1_stamp = pose_buffer_[1].header.stamp;
  double dt = t1_stamp.seconds() - t0_stamp.seconds();
  geometry_msgs::msg::TwistStamped twist;
  twist.header.frame_id = robot_frame_id;
  twist.header.stamp = pose_buffer_[1].header.stamp;
  geometry_msgs::msg::Quaternion rot =
    quaternion_operation::getRotation(
    pose_buffer_[0].pose.orientation,
    pose_buffer_[1].pose.orientation);
  twist.twist.angular = quaternion_operation::convertQuaternionToEulerAngle(rot);
  twist.twist.angular.x = twist.twist.angular.x / dt;
  twist.twist.angular.y = twist.twist.angular.y / dt;
  twist.twist.angular.z = twist.twist.angular.z / dt;
  Eigen::Matrix3d rotation_mat = quaternion_operation::getRotationMatrix(
    quaternion_operation::conjugate(pose_buffer_[0].pose.orientation));
  Eigen::Vector3d trans_vec;
  trans_vec[0] = pose_buffer_[1].pose.position.x - pose_buffer_[0].pose.position.x;
  trans_vec[1] = pose_buffer_[1].pose.position.y - pose_buffer_[0].pose.position.y;
  trans_vec[2] = pose_buffer_[1].pose.position.z - pose_buffer_[0].pose.position.z;
  trans_vec = rotation_mat * trans_vec;
  twist.twist.linear.x = trans_vec[0] / dt;
  twist.twist.linear.y = trans_vec[1] / dt;
  twist.twist.linear.z = trans_vec[2] / dt;
  return twist;
}
}  // namespace pf_localization
