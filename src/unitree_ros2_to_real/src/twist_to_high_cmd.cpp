#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace UNITREE_LEGGED_SDK;

class Converter : public rclcpp::Node
{
  public:
    Converter()
    : Node("converter_to_high_cmd")
    {
      subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&Converter::cmd_vel_callback, this, _1));
      publisher_ = this->create_publisher<ros2_unitree_legged_msgs::msg::HighCmd>(
        "high_cmd", 1);
      // publisher_ = this->create_publisher<std_msgs::msg::String>(
      //   "high_cmd", 1);
      timer_ = this->create_wall_timer(500ms, std::bind(&Converter::timer_callback, this));
    }

  private:
    // rclcpp::Node::SharedPtr nh_;

    ros2_unitree_legged_msgs::msg::HighCmd high_cmd_msg;

    high_cmd_msg.head[0] = 0xFE;
    high_cmd_msg.head[1] = 0xEF;
    high_cmd_msg.level_flag = HIGHLEVEL;
    high_cmd_msg.mode = 0;
    high_cmd_msg.gait_type = 0;
    high_cmd_msg.speed_level = 0;
    high_cmd_msg.foot_raise_height = 0;
    high_cmd_msg.body_height = 0;
    high_cmd_msg.euler[0] = 0;
    high_cmd_msg.euler[1] = 0;
    high_cmd_msg.euler[2] = 0;
    high_cmd_msg.velocity[0] = 0.0f;
    high_cmd_msg.velocity[1] = 0.0f;
    high_cmd_msg.yaw_speed = 0.0f;
    high_cmd_msg.reserve = 0;

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) // const
    {
      high_cmd_msg.velocity[0] = msg->linear.x;
      high_cmd_msg.velocity[1] = msg->linear.y;
      high_cmd_msg.yaw_speed = msg->angular.z;

      RCLCPP_INFO(this->get_logger(), "I heard: Linear ['%f', '%f', '%f'] and Angular ['%f', '%f', '%f']",
        msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z);
    }

    void timer_callback()
    {
      // auto message = std_msgs::msg::String();
      // message.data = "Linear (" + std::to_string(linearX) + ", " + std::to_string(linearY) + ", " + std::to_string(linearZ) + ") and Angular (" + std::to_string(angularZ) + ")";

      publisher_->publish(high_cmd_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr publisher_;
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<Converter>());

  rclcpp::shutdown();
  return 0;
}