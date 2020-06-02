#include <pf_localization/pf_localization_component.h>
#include <rclcpp_components/register_node_macro.hpp>

namespace pf_localization
{
PfLocalizationComponent::PfLocalizationComponent(const rclcpp::NodeOptions & options)
: Node("pf_localization_node", options), tf_broadcaster_(this), static_tf_broadcaster_(this),
  tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
{
  pose_recieved_ = false;
  declare_parameter("num_particles", 100);
  get_parameter("num_particles", num_particles_);
  declare_parameter("pose_topic", "/gps/fix/position");
  get_parameter("pose_topic", pose_topic_);
  declare_parameter("twist_topic", "/twist");
  get_parameter("twist_topic", twist_topic_);
  declare_parameter("map_frame", "map");
  get_parameter("map_frame", map_frame_);
  declare_parameter("base_link_frame", "base_link");
  get_parameter("base_link_frame", base_link_frame_);
  declare_parameter("publish_frame", false);
  get_parameter("publish_frame", publish_frame_);
  declare_parameter("estimate_3d_pose", false);
  get_parameter("estimate_3d_pose", estimate_3d_pose_);
  declare_parameter("publish_marker", false);
  get_parameter("publish_marker", publish_marker_);
  declare_parameter("expansion_reset_ess_threashold", 10);
  get_parameter("expansion_reset_ess_threashold", expansion_reset_ess_threashold_);
  declare_parameter("max_expantion_orientation", 0.5);
  get_parameter("max_expantion_orientation", max_expantion_orientation_);
  declare_parameter("max_expantion_position", 0.5);
  get_parameter("max_expantion_position", max_expantion_position_);
  declare_parameter("sensor_reset_ess_threashold", 30);
  get_parameter("sensor_reset_ess_threashold", sensor_reset_ess_threashold_);
  declare_parameter("max_sensor_reset_orientation", 0.5);
  get_parameter("max_sensor_reset_orientation", max_sensor_reset_orientation_);
  declare_parameter("max_sensor_reset_position", 0.5);
  get_parameter("max_sensor_reset_position", max_sensor_reset_position_);
  declare_parameter("sensor_reset_radius", 5.0);
  get_parameter("sensor_reset_radius", sensor_reset_radius_);
  declare_parameter("weight_position", 0.9);
  get_parameter("weight_position", weight_position_);
  declare_parameter("weight_orientation", 0.1);
  get_parameter("weight_orientation", weight_orientation_);
  assert(num_particles_ > 0);
  assert(expansion_reset_ess_threashold_ > 0);
  assert(sensor_reset_ess_threashold_ > 0);
  assert(sensor_reset_ess_threashold_ < expansion_reset_ess_threashold_);
  assert(sensor_reset_radius_ > 0);
  pf_ptr_ = std::make_shared<ParticleFilter>(num_particles_, 1, estimate_3d_pose_,
      expansion_reset_ess_threashold_, max_expantion_orientation_, max_expantion_position_,
      sensor_reset_ess_threashold_, max_sensor_reset_orientation_, max_sensor_reset_position_,
      sensor_reset_radius_,
      weight_position_, weight_orientation_, get_clock());
  current_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("current_pose", 1);
  current_twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("current_twist", 1);
  ess_pub_ = this->create_publisher<std_msgs::msg::Float32>("~/effective_sample_size", 1);
  if (publish_marker_) {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/marker", 1);
  }
  twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    twist_topic_, 1,
    std::bind(&PfLocalizationComponent::twistStampedCallback, this, std::placeholders::_1));
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic_, 1,
    std::bind(&PfLocalizationComponent::poseStampedCallback, this, std::placeholders::_1));
}

PfLocalizationComponent::~PfLocalizationComponent()
{

}

void PfLocalizationComponent::run()
{
  using namespace std::chrono_literals;
  update_pose_timer_ =
    this->create_wall_timer(30ms, std::bind(&PfLocalizationComponent::updateCurrentPose, this));
}

boost::optional<geometry_msgs::msg::TwistStamped> PfLocalizationComponent::getCurrentTwist()
{
  return pf_ptr_->getCurrentTwist();
}

void PfLocalizationComponent::updateCurrentPose()
{
  mtx_.lock();
  rclcpp::Time now = get_clock()->now();
  boost::optional<geometry_msgs::msg::PoseStamped> current_pose = pf_ptr_->estimateCurrentPose(now);
  if (current_pose) {
    if (publish_frame_) {
      broadcastInitialPoseFrame(now);
      broadcastBaseLinkFrame(now, *current_pose);
    }
    current_pose_pub_->publish(*current_pose);
    std_msgs::msg::Float32 ess_msg;
    ess_msg.data = pf_ptr_->getEffectiveSampleSize();
    ess_pub_->publish(ess_msg);
  }
  boost::optional<geometry_msgs::msg::TwistStamped> current_twist = getCurrentTwist();
  if (current_twist) {
    current_twist_pub_->publish(*current_twist);
  }
  if (publish_marker_) {
    visualization_msgs::msg::MarkerArray marker;
    std::vector<Particle> particles = pf_ptr_->getParticles();
    int id = 0;
    for (auto itr = particles.begin(); itr != particles.end(); itr++) {
      visualization_msgs::msg::Marker single_marker;
      single_marker.header = itr->pose.header;
      single_marker.pose = itr->pose.pose;
      single_marker.type = single_marker.ARROW;
      single_marker.id = id;
      single_marker.color.r = 1.0 - itr->weight;
      single_marker.color.g = 0.0;
      single_marker.color.b = itr->weight;
      single_marker.color.a = 1.0;
      single_marker.action = single_marker.ADD;
      single_marker.frame_locked = true;
      single_marker.scale.x = 1.0;
      single_marker.scale.y = 0.1;
      single_marker.scale.z = 0.1;
      single_marker.ns = "marker" + std::to_string(id);
      marker.markers.push_back(single_marker);
      id++;
    }
    marker_pub_->publish(marker);
  }
}

void PfLocalizationComponent::broadcastBaseLinkFrame(
  rclcpp::Time stamp,
  geometry_msgs::msg::PoseStamped pose)
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = map_frame_;
  transform_stamped.header.stamp = stamp;
  transform_stamped.child_frame_id = base_link_frame_;
  transform_stamped.transform.translation.x = pose.pose.position.x;
  transform_stamped.transform.translation.y = pose.pose.position.y;
  transform_stamped.transform.translation.z = pose.pose.position.z;
  transform_stamped.transform.rotation = pose.pose.orientation;
  tf_broadcaster_.sendTransform(transform_stamped);
}

void PfLocalizationComponent::broadcastInitialPoseFrame(rclcpp::Time stamp)
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = map_frame_;
  transform_stamped.header.stamp = stamp;
  transform_stamped.child_frame_id = "initial_pose";
  transform_stamped.transform.translation.x = initial_pose_.pose.position.x;
  transform_stamped.transform.translation.y = initial_pose_.pose.position.y;
  transform_stamped.transform.translation.z = initial_pose_.pose.position.z;
  transform_stamped.transform.rotation.x = initial_pose_.pose.orientation.x;
  transform_stamped.transform.rotation.y = initial_pose_.pose.orientation.y;
  transform_stamped.transform.rotation.z = initial_pose_.pose.orientation.z;
  transform_stamped.transform.rotation.w = initial_pose_.pose.orientation.w;
  static_tf_broadcaster_.sendTransform(transform_stamped);
}

void PfLocalizationComponent::twistStampedCallback(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  mtx_.lock();
  pf_ptr_->updateTwist(*msg);
  mtx_.unlock();
}

void PfLocalizationComponent::poseStampedCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  mtx_.lock();
  if (!pose_recieved_) {
    initial_pose_ = *msg;
    pf_ptr_->setInitialPose(*msg);
    pose_recieved_ = true;
  }
  pf_ptr_->updatePose(*msg);
  mtx_.unlock();
}

template<class C>
boost::optional<C> PfLocalizationComponent::transformToMapFrame(C input)
{
  if (input.header.frame_id != map_frame_) {
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_.lookupTransform(map_frame_, input.header.frame_id,
          input.header.stamp, rclcpp::Duration(0.3));
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(get_logger(), ex.what());
      return boost::none;
    }
    tf2::doTransform(input, input, transform_stamped);
  }
  return input;
}
}

RCLCPP_COMPONENTS_REGISTER_NODE(pf_localization::PfLocalizationComponent)
