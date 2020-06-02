// Headers in ROS
#include <rclcpp/rclcpp.hpp>

// Headers in this package
#include <pf_localization/pf_localization_component.hpp>

// Headers in GLOG
#include <glog/logging.h>

int main(int argc, char * argv[])
{
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<pf_localization::PfLocalizationComponent>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
