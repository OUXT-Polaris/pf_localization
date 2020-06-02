#ifndef PF_LOCALIZATION_PF_LOCALIZATION_H_INCLUDED
#define PF_LOCALIZATION_PF_LOCALIZATION_H_INCLUDED

// Headers in this package
#include <pf_localization/particle_filter.h>

// Headers in ROS
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Headers in STL
#include <memory>
#include <mutex>

// Headers in Boost
#include <boost/optional.hpp>
#include <boost/thread.hpp>

namespace pf_localization
{
class PfLocalizationComponent : public rclcpp::Node
{
public:
  PfLocalizationComponent(const rclcpp::NodeOptions & options);
  ~PfLocalizationComponent();
  void run();

private:
  void updateCurrentPose();
  boost::optional<geometry_msgs::msg::TwistStamped> getCurrentTwist();
  void twistStampedCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void poseStampedCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  std::mutex mtx_;
  std::shared_ptr<ParticleFilter> pf_ptr_;
  int num_particles_;
  std::string pose_topic_;
  std::string twist_topic_;
  std::string map_frame_;
  std::string base_link_frame_;
  int update_rate_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  bool pose_recieved_;
  bool estimate_3d_pose_;
  double expansion_reset_ess_threashold_;
  double max_expantion_orientation_;
  double max_expantion_position_;
  double sensor_reset_ess_threashold_;
  double max_sensor_reset_orientation_;
  double max_sensor_reset_position_;
  double sensor_reset_radius_;
  double weight_position_;
  double weight_orientation_;
  void broadcastBaseLinkFrame(rclcpp::Time stamp, geometry_msgs::msg::PoseStamped pose);
  void broadcastInitialPoseFrame(rclcpp::Time stamp);
  template<class C>
  boost::optional<C> transformToMapFrame(C input);
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr current_twist_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ess_pub_;
  bool publish_marker_;
  bool publish_frame_;
  geometry_msgs::msg::PoseStamped initial_pose_;
  rclcpp::TimerBase::SharedPtr update_pose_timer_;
};
}
#endif  //PF_LOCALIZATION_PF_LOCALIZATION_H_INCLUDED
