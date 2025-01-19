/**
 * @function flight_conversion_node
 */
#include <conversion_tool/flight_conversion.h>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // Create conversion tool
  auto flight_conversion = std::make_shared<FlightConversion>();

  // Initialize
  if ( !flight_conversion->parseConfigParams() )
    return 1;

  if ( !flight_conversion->initCommunication() )
    return 1;

  rclcpp::spin(flight_conversion);

  rclcpp::shutdown();
}
