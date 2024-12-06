
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <edoras_msgs/msg/test_message1.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

class TestPublisher : public rclcpp::Node
{
    public:
   TestPublisher() : Node("test_publisher")
   {

    std_msgs::msg::Header hdr;
        RCLCPP_INFO(this->get_logger(), "Size of time: %d \n", sizeof(hdr.stamp));
            RCLCPP_INFO(this->get_logger(), "Size of frame id: %d \n", sizeof(hdr.frame_id));
    RCLCPP_INFO(this->get_logger(), "Size of header: %d \n", sizeof(hdr));
    publisher_twist_ = this->create_publisher<geometry_msgs::msg::Twist>("twist_command", 10);
    publisher_string_ = this->create_publisher<std_msgs::msg::String>("edoras_set_mode_command", 10);
    subscriber_js_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_state_telemetry", 10, std::bind(&TestPublisher::sub_cb, this, std::placeholders::_1));
    
    auto timer_callback = 
      [this]() -> void {

        auto message = geometry_msgs::msg::Twist();
        RCLCPP_INFO(this->get_logger(), "Publishing message twist");
        this->publisher_twist_->publish(message);

        auto msg2 = std_msgs::msg::String();
        RCLCPP_INFO(this->get_logger(), "Publishing message string");
        this->publisher_string_->publish(msg2);

      };

      timer_ = this->create_wall_timer(500ms, timer_callback);
    }

    void sub_cb(const sensor_msgs::msg::JointState::SharedPtr _msg)
    {
      RCLCPP_INFO(this->get_logger(), "\t ** Got a sensor message back!");
    }

  protected:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_twist_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_string_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_js_;
  size_t count_;

};


int main(int argc, char* argv[])
{
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<TestPublisher>());
   rclcpp::shutdown();
   return 0;
}
