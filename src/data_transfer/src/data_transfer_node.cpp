/**
 * @function data_transfer_node.cpp
 */
#include <data_transfer/data_transfer.h>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // Create conversion tool
  auto data_transfer = std::make_shared<DataTransfer>();

  // Initialize
  if ( !data_transfer->parseConfigParams() )
    return 1;

  if ( !data_transfer->initialize() )
    return 1;

  rclcpp::spin(data_transfer);
  rclcpp::shutdown();
}
