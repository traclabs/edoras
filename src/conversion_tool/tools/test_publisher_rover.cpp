
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>

using namespace std::chrono_literals;

class TestPublisher : public rclcpp::Node
{
    public:
   TestPublisher() : Node("test_publisher")
   {
    count_ = 0;
    publisher_twist_ = this->create_publisher<geometry_msgs::msg::Twist>("command_twist", 10);
    subscriber_pose_ = this->create_subscription<geometry_msgs::msg::Pose>("telemetry_pose", 10, std::bind(&TestPublisher::sub_cb, this, std::placeholders::_1));
    
    auto timer_callback = 
      [this]() -> void {

        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 0.5;
        message.linear.y = 0.0;
        message.linear.z = 0.0;
        message.angular.x = 0.0;
        message.angular.y = 0.0;
        message.angular.z = -0.3;
        this->publisher_twist_->publish(message);

        count_++;
      };

      timer_ = this->create_wall_timer(500ms, timer_callback);
    }

    void sub_cb(const geometry_msgs::msg::Pose::SharedPtr _msg)
    {
      RCLCPP_INFO(this->get_logger(), "\t ** Got a Pose message back!: %f %f %f", _msg->position.x, _msg->position.y, _msg->position.z);
    }

  protected:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_twist_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriber_pose_;
  size_t count_;

};


int main(int argc, char* argv[])
{
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<TestPublisher>());
   rclcpp::shutdown();
   return 0;
}
