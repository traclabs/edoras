#include <conversion_tool/ground_conversion.h>

int main(int argc, char* argv[])
{
  // Init
  rclcpp::init(argc, argv);

  // Create conversion tool
  auto ground_conversion = std::make_shared<GroundConversion>();
  ground_conversion->parseConfigParams(); 
  rclcpp::spin(ground_conversion);


  // Add subscription on the fly
  
  rclcpp::shutdown();
}