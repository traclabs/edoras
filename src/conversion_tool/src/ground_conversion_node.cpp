/**
 * @function ground_conversion_node
 */
#include <conversion_tool/ground_conversion.h>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // Create conversion tool
  auto ground_conversion = std::make_shared<GroundConversion>();

  // Initialize
  RCLCPP_INFO(ground_conversion->get_logger(), "Parse config params \n");
  if ( !ground_conversion->parseConfigParams() )
    return 1;
  RCLCPP_INFO(ground_conversion->get_logger(), "Init communication \n");    
  if ( !ground_conversion->initCommunication() )
    return 1;
  RCLCPP_INFO(ground_conversion->get_logger(), "About to spin \n");      
  rclcpp::spin(ground_conversion);

  rclcpp::shutdown();
}
