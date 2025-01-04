
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
    publisher_pose_ = this->create_publisher<geometry_msgs::msg::Pose>("/command_pose", 10);
    //subscriber_pose_ = this->create_subscription<geometry_msgs::msg::Pose>("telemetry_pose", 10, std::bind(&TestPublisher::sub_cb, this, std::placeholders::_1));
    

    }

    void publishCmd()
    {
        auto message = geometry_msgs::msg::Pose();
        message.position.x = 0.442;
        message.position.y = 4.248;
        message.position.z = 5.209;
        message.orientation.x = 0.851;
        message.orientation.y = 0.055;
        message.orientation.z = 0.039;
        message.orientation.w = 0.521;
        
        this->publisher_pose_->publish(message);
    }

    void sub_cb(const geometry_msgs::msg::Pose::SharedPtr _msg)
    {
      RCLCPP_INFO(this->get_logger(), "\t ** Got a Pose message back!: %f %f %f", _msg->position.x, _msg->position.y, _msg->position.z);
    }

  protected:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_pose_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriber_pose_;
  size_t count_;

};


int main(int argc, char* argv[])
{
   rclcpp::init(argc, argv);
   auto node = std::make_shared<TestPublisher>();
   node->publishCmd();
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
}
