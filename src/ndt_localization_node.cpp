#include <ndt_localization/ndt_localization.h>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<NDTLocalization>());

  rclcpp::shutdown();

  return 0;
}
