#include <pf_localization/particle_filter.hpp>

#include <quaternion_operation/quaternion_operation.h>

namespace pf_localization
{
ParticleFilter::ParticleFilter(
  int num_particles, double buffer_length, bool estimate_3d_pose,
  double expansion_reset_ess_threashold, double max_expansion_orientation,
  double max_expantion_position,
  double sensor_reset_ess_threashold, double max_sensor_reset_orientation,
  double max_sensor_reset_position, double sensor_reset_radius,
  double weight_position, double weight_orientation, rclcpp::Clock::SharedPtr clock)
: num_particles(num_particles), buffer_length(buffer_length), estimate_3d_pose(estimate_3d_pose),
  expansion_reset_ess_threashold(expansion_reset_ess_threashold),
  max_expansion_orientation(max_expansion_orientation), max_expantion_position(
    max_expantion_position),
  sensor_reset_ess_threashold(sensor_reset_ess_threashold), max_sensor_reset_orientation(
    max_sensor_reset_orientation),
  max_sensor_reset_position(max_sensor_reset_position), sensor_reset_radius(sensor_reset_radius),
  weight_position(weight_position), weight_orientation(weight_orientation), engine_(seed_gen_()),
  position_dist_(1.0, 0.1), rotation_dist_(1.0, 0.1), mt_(seed_gen_()), uniform_dist_(0.0, 1.0),
  pose_buf_(clock, "/pose", buffer_length), twist_buf_(clock, "/twist", buffer_length)
{
  particles_ = std::vector<Particle>(num_particles);
  current_pose_ = boost::none;
  initial_pose_ = boost::none;
  twist_estimator_ = std::unique_ptr<TwistEstimator>(new TwistEstimator("base_link"));
}

ParticleFilter::~ParticleFilter()
{

}

boost::optional<geometry_msgs::msg::PoseStamped> ParticleFilter::getInitialPose()
{
  return initial_pose_;
}

void ParticleFilter::updateTwist(geometry_msgs::msg::TwistStamped twist)
{
  twist_buf_.addData(twist);
}

void ParticleFilter::updatePose(geometry_msgs::msg::PoseStamped pose)
{
  pose_buf_.addData(pose);
}

void ParticleFilter::setInitialPose(geometry_msgs::msg::PoseStamped pose)
{
  current_pose_ = pose;
  initial_pose_ = pose;
  for (auto itr = particles_.begin(); itr != particles_.end(); itr++) {
    itr->weight = (double)1.0 / (double)num_particles;
    itr->pose = pose;
  }
}

void ParticleFilter::sensorReset(geometry_msgs::msg::PoseStamped pose)
{
  //ROS_INFO_STREAM("execute sensor reset");
  std::random_device rd;
  std::mt19937 mt(rd());
  std::uniform_real_distribution<double> rand(-1.0, 1.0);
  for (auto itr = particles_.begin(); itr != particles_.end(); itr++) {
    geometry_msgs::msg::Vector3 rand_rpy;
    geometry_msgs::msg::Point rand_xyz;
    if (estimate_3d_pose) {
      rand_rpy.x = rand(mt) * max_expansion_orientation;
      rand_rpy.y = rand(mt) * max_expansion_orientation;
      rand_rpy.z = rand(mt) * max_expansion_orientation;
      rand_xyz.x = rand(mt) * max_expantion_position;
      rand_xyz.y = rand(mt) * max_expantion_position;
      rand_xyz.z = rand(mt) * max_expantion_position;
    } else {
      rand_rpy.x = 0;
      rand_rpy.y = 0;
      rand_rpy.z = rand(mt) * max_expansion_orientation;
      rand_xyz.x = rand(mt) * max_expantion_position;
      rand_xyz.y = rand(mt) * max_expantion_position;
      rand_xyz.z = 0;
    }
    geometry_msgs::msg::Quaternion rand_quat = quaternion_operation::convertEulerAngleToQuaternion(
      rand_rpy);
    itr->pose.pose.orientation = pose.pose.orientation * rand_quat;
    itr->pose.pose.position.x = pose.pose.position.x + rand_xyz.x;
    itr->pose.pose.position.y = pose.pose.position.y + rand_xyz.y;
    itr->pose.pose.position.z = pose.pose.position.z + rand_xyz.z;
    itr->weight = (double)1.0 / (double)num_particles;
  }
}

void ParticleFilter::expansionReset()
{
  //ROS_INFO_STREAM("execute expansion reset");
  std::random_device rd;
  std::mt19937 mt(rd());
  std::uniform_real_distribution<double> rand(-1.0, 1.0);
  for (auto itr = particles_.begin(); itr != particles_.end(); itr++) {
    geometry_msgs::msg::Vector3 rand_rpy;
    geometry_msgs::msg::Point rand_xyz;
    if (estimate_3d_pose) {
      rand_rpy.x = rand(mt) * max_expansion_orientation;
      rand_rpy.y = rand(mt) * max_expansion_orientation;
      rand_rpy.z = rand(mt) * max_expansion_orientation;
      rand_xyz.x = rand(mt) * max_expantion_position;
      rand_xyz.y = rand(mt) * max_expantion_position;
      rand_xyz.z = rand(mt) * max_expantion_position;
    } else {
      rand_rpy.x = 0;
      rand_rpy.y = 0;
      rand_rpy.z = rand(mt) * max_expansion_orientation;
      rand_xyz.x = rand(mt) * max_expantion_position;
      rand_xyz.y = rand(mt) * max_expantion_position;
      rand_xyz.z = 0;
    }
    geometry_msgs::msg::Quaternion rand_quat = quaternion_operation::convertEulerAngleToQuaternion(
      rand_rpy);
    itr->pose.pose.orientation = itr->pose.pose.orientation * rand_quat;
    itr->pose.pose.position.x = itr->pose.pose.position.x + rand_xyz.x;
    itr->pose.pose.position.y = itr->pose.pose.position.y + rand_xyz.y;
    itr->pose.pose.position.z = itr->pose.pose.position.z + rand_xyz.z;
    itr->weight = (double)1.0 / (double)num_particles;
  }
}

bool ParticleFilter::checkQuaternion(geometry_msgs::msg::Quaternion quat)
{
  double a =
    std::sqrt(std::pow(quat.x,
      2) + std::pow(quat.y, 2) + std::pow(quat.z, 2) + std::pow(quat.w, 2));
  double b = 1.0;
  if (fabs(a - b) < DBL_EPSILON) {
    return true;
  }
  return false;
}

void ParticleFilter::normalizeWeights()
{
  double sum_weights = getTotalWeights();
  for (auto itr = particles_.begin(); itr != particles_.end(); itr++) {
    itr->weight = itr->weight / sum_weights;
  }
}

double ParticleFilter::getTotalWeights()
{
  double ret = 0.0;
  for (auto itr = particles_.begin(); itr != particles_.end(); itr++) {
    ret = ret + itr->weight;
  }
  return ret;
}

double ParticleFilter::getEffectiveSampleSize()
{
  double squared_sum = 0.0;
  for (auto itr = particles_.begin(); itr != particles_.end(); itr++) {
    squared_sum = squared_sum + (itr->weight * itr->weight);
  }
  return 1 / squared_sum;
}

void ParticleFilter::resampling()
{
  std::vector<Particle> new_particles(num_particles);
  double total_weight = getTotalWeights();
  double accum = uniform_dist_(mt_) / (double)num_particles;
  double step = total_weight / (double)num_particles;
  std::vector<int> choice(num_particles);
  for (auto & c : choice) {
    int j = 0;
    double current_total_weight = 0.0;
    while (current_total_weight <= accum && j < (num_particles - 1)) {
      current_total_weight = current_total_weight + particles_[j].weight;
      j++;
    }
    c = j;
    accum = accum + step;
  }
  for (int i = 0; i < num_particles; i++) {
    new_particles[i] = particles_[choice[i]];
    new_particles[i].weight = 1.0 / num_particles;
  }
  particles_ = new_particles;
}

boost::optional<geometry_msgs::msg::PoseStamped> ParticleFilter::estimateCurrentPose(
  rclcpp::Time stamp)
{
  geometry_msgs::msg::TwistStamped twist;
  bool twist_query_succeed = twist_buf_.queryData(stamp, twist);
  geometry_msgs::msg::PoseStamped pose;
  bool pose_query_succeed = pose_buf_.queryData(stamp, pose);
  if (twist_query_succeed && pose_query_succeed && current_pose_) {
    rclcpp::Time current_pose_stamp = current_pose_->header.stamp;
    double duration = stamp.seconds() - current_pose_stamp.seconds();
    for (auto itr = particles_.begin(); itr != particles_.end(); itr++) {
      // Transition
      geometry_msgs::msg::Vector3 orientation;
      if (estimate_3d_pose) {
        orientation.x = twist.twist.angular.x * duration * rotation_dist_(engine_);
        orientation.y = twist.twist.angular.y * duration * rotation_dist_(engine_);
      } else {
        orientation.x = 0.0;
        orientation.y = 0.0;
      }
      orientation.z = twist.twist.angular.z * duration * rotation_dist_(engine_);
      geometry_msgs::msg::Quaternion twist_angular_quat =
        quaternion_operation::convertEulerAngleToQuaternion(orientation);
      itr->pose.pose.orientation = quaternion_operation::rotation(itr->pose.pose.orientation,
          twist_angular_quat);
      Eigen::Vector3d trans_vec;
      trans_vec(0) = twist.twist.linear.x * duration * position_dist_(engine_);
      trans_vec(1) = twist.twist.linear.y * duration * position_dist_(engine_);
      if (estimate_3d_pose) {
        trans_vec(2) = twist.twist.linear.z * duration * position_dist_(engine_);
      } else {
        trans_vec(2)  = 0;
      }
      Eigen::Matrix3d rotation_mat = quaternion_operation::getRotationMatrix(
        itr->pose.pose.orientation);
      trans_vec = rotation_mat * trans_vec;
      itr->pose.pose.position.x = itr->pose.pose.position.x + trans_vec(0);
      itr->pose.pose.position.y = itr->pose.pose.position.y + trans_vec(1);
      if (estimate_3d_pose) {
        itr->pose.pose.position.z = itr->pose.pose.position.z + trans_vec(2);
      }
      // Evaluate
      double dist = std::sqrt(std::pow(itr->pose.pose.position.x - pose.pose.position.x, 2) +
          std::pow(itr->pose.pose.position.y - pose.pose.position.y, 2) +
          std::pow(itr->pose.pose.position.z - pose.pose.position.z, 2));
      geometry_msgs::msg::Quaternion diff_quat = quaternion_operation::getRotation(
        itr->pose.pose.orientation, pose.pose.orientation);
      geometry_msgs::msg::Vector3 diff_vec = quaternion_operation::convertQuaternionToEulerAngle(
        diff_quat);
      double diff_angle = std::sqrt(
        diff_vec.x * diff_vec.x + diff_vec.y * diff_vec.y + diff_vec.z * diff_vec.z);
      // avoid zero diveide
      if (dist < 0.01) {
        dist = 0.01;
      }
      if (diff_angle < 0.01) {
        diff_angle = 0.01;
      }
      //itr->weight = 1/(dist*diff_angle);
      itr->weight = 1 / (weight_position * dist + weight_orientation * diff_angle);
    }
    double total_weight = 0.0;
    double heighest_weight = 0;
    geometry_msgs::msg::PoseStamped ret = particles_[0].pose;
    for (auto itr = particles_.begin(); itr != particles_.end(); itr++) {
      total_weight = total_weight + itr->weight;
      if (heighest_weight < itr->weight) {
        heighest_weight = itr->weight;
        ret = itr->pose;
      }
    }
    // Resampling
    normalizeWeights();
    resampling();
    ret.header.stamp = stamp;
    current_pose_ = ret;
    double dist = std::sqrt(std::pow(ret.pose.position.x - pose.pose.position.x, 2) +
        std::pow(ret.pose.position.y - pose.pose.position.y, 2) +
        std::pow(ret.pose.position.z - pose.pose.position.z, 2));
    //Reset
    double ess = getEffectiveSampleSize();
    if (ess < sensor_reset_ess_threashold) {
      twist_estimator_->clear();
      sensorReset(pose);
    } else if (ess < expansion_reset_ess_threashold) {
      twist_estimator_->clear();
      expansionReset();
    } else if (dist > sensor_reset_radius) {
      twist_estimator_->clear();
      sensorReset(pose);
    }
    twist_estimator_->add(ret);
    return ret;
  }
  return boost::none;
}
}
