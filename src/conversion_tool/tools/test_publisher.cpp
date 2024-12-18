
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <edoras_msgs/msg/test_message1.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>

using namespace std::chrono_literals;

class TestPublisher : public rclcpp::Node
{
    public:
   TestPublisher() : Node("test_publisher")
   {
    count_ = 0;
    std_msgs::msg::Header hdr;
        RCLCPP_INFO(this->get_logger(), "Size of time: %d \n", sizeof(hdr.stamp));
            RCLCPP_INFO(this->get_logger(), "Size of frame id: %d \n", sizeof(hdr.frame_id));
    RCLCPP_INFO(this->get_logger(), "Size of header: %d \n", sizeof(hdr));
    publisher_pose_ = this->create_publisher<geometry_msgs::msg::Pose>("pose_command", 10);
    publisher_js_ = this->create_publisher<sensor_msgs::msg::JointState>("js_command", 10);
    subscriber_js_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_state_telemetry", 10, std::bind(&TestPublisher::sub_cb, this, std::placeholders::_1));
    
    auto timer_callback = 
      [this]() -> void {

        auto message = geometry_msgs::msg::Pose();
        message.position.x = 23;
        message.position.y = 29;
        message.position.z = 12;
        message.orientation.w = 1985;
        //RCLCPP_INFO(this->get_logger(), "Publishing message Pose");
        this->publisher_pose_->publish(message);

        sleep(1);
        auto msg2 = sensor_msgs::msg::JointState();
        if(count_ % 2 == 0) 
        {
          msg2.name = {"j1", "j2", "j3", "j4", "j5"};
          msg2.position = {0.2, 0.3, 0.4, 0.5, 0.6};
        }
        else 
        {
          msg2.name = {"b1", "b2", "b3"};
          msg2.position = {0.42, 0.51, 0.68};        
        }  
        //RCLCPP_INFO(this->get_logger(), "Publishing message Joint State");
        //this->publisher_js_->publish(msg2);


        count_++;
      };

      timer_ = this->create_wall_timer(500ms, timer_callback);
    }

    void sub_cb(const sensor_msgs::msg::JointState::SharedPtr _msg)
    {
      RCLCPP_INFO(this->get_logger(), "\t ** Got a sensor message back!");
    }

  protected:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_pose_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_js_;
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
