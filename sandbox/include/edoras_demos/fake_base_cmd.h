
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

class FakeBaseCmd : public rclcpp::Node
{
   public: 
   explicit FakeBaseCmd();
   bool init();
   void spin();

   protected:
   
   void vel_cmd_cb(const geometry_msgs::msg::Twist::SharedPtr _msg);
   
   geometry_msgs::msg::Twist vel_msg_;
   rclcpp::Time vel_msg_stamp_;

   std::mutex vel_mux_;

   double x_, y_, z_, ra_, pa_, ya_;
   std::string base_link_frame_, fixed_frame_, cmd_vel_topic_;
   tf2::Transform tf_robot_reference_, base_to_robot_ref_tf_, tf_base_;
   
   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_sub_;

   std::shared_ptr<tf2_ros::StaticTransformBroadcaster> br_;
   std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
   std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

};
