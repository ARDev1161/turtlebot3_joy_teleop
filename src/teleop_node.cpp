#include <memory>
#include <stdexcept>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

class TeleopNode : public rclcpp::Node
{
public:
  TeleopNode()
  : Node("turtlebot3_joy_teleop")
  {
    // Declare parameters
    this->declare_parameter("enable_button", 5);
    this->declare_parameter("axis_linear", 1);
    this->declare_parameter("axis_angular", 0);
    this->declare_parameter("scale_linear", 0.5);
    this->declare_parameter("scale_angular", 0.5);
    this->declare_parameter("require_enable", true);

    // Get parameters
    enable_button_ = this->get_parameter("enable_button").as_int();
    axis_linear_   = this->get_parameter("axis_linear").as_int();
    axis_angular_  = this->get_parameter("axis_angular").as_int();
    scale_linear_  = this->get_parameter("scale_linear").as_double();
    scale_angular_ = this->get_parameter("scale_angular").as_double();
    require_enable_= this->get_parameter("require_enable").as_bool();

    // Publisher for /cmd_vel
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Subscribers for joystick and spacenav
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10,
      std::bind(&TeleopNode::joy_callback, this, std::placeholders::_1)
    );
    spacenav_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "spacenav/joy", 10,
      std::bind(&TeleopNode::joy_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Teleop C++ node started");
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // If enable button required but not pressed, skip
    if (require_enable_)
    {
      if (enable_button_ >= static_cast<int>(msg->buttons.size()) || msg->buttons[enable_button_] == 0)
        return;
    }

    auto twist = geometry_msgs::msg::Twist();
    if (axis_linear_ < static_cast<int>(msg->axes.size())) {
      twist.linear.x = msg->axes[axis_linear_] * scale_linear_;
    }
    if (axis_angular_ < static_cast<int>(msg->axes.size())) {
      twist.angular.z = msg->axes[axis_angular_] * scale_angular_;
    }

    pub_->publish(twist);
  }

  // Parameters
  int enable_button_;
  int axis_linear_;
  int axis_angular_;
  double scale_linear_;
  double scale_angular_;
  bool require_enable_;

  // ROS interfaces
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr spacenav_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TeleopNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
