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
  if ( !ground_conversion->parseConfigParams() )
    return 1;
    
  if ( !ground_conversion->initCommunication() )
    return 1;
      
  rclcpp::spin(ground_conversion);

  rclcpp::shutdown();
}
