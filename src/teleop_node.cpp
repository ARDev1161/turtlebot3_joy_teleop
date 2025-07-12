#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class TeleopNode : public rclcpp::Node
{
public:
  TeleopNode() : Node("turtlebot3_joy_teleop")
  {
    // Declare parameters
    declare_parameter("joy_topic",        "joy");
    declare_parameter("spacenav_topic",   "spacenav/joy");
    declare_parameter("cmd_vel_topic",    "cmd_vel");
    declare_parameter("use_timestamp",    true);

    declare_parameter("enable_button", 5);
    declare_parameter("axis_linear",   1);
    declare_parameter("axis_angular",  0);
    declare_parameter("scale_linear",  0.5);
    declare_parameter("scale_angular", 0.5);
    declare_parameter("require_enable", true);

    // Read parameters
    std::string joy_topic      = get_parameter("joy_topic").as_string();
    std::string spacenav_topic = get_parameter("spacenav_topic").as_string();
    std::string cmd_vel_topic  = get_parameter("cmd_vel_topic").as_string();
    bool use_timestamp         = get_parameter("use_timestamp").as_bool();

    enable_button_  = get_parameter("enable_button").as_int();
    axis_linear_    = get_parameter("axis_linear").as_int();
    axis_angular_   = get_parameter("axis_angular").as_int();
    scale_linear_   = get_parameter("scale_linear").as_double();
    scale_angular_  = get_parameter("scale_angular").as_double();
    require_enable_ = get_parameter("require_enable").as_bool();

    // Publishers
    if (use_timestamp) {
      twist_stamped_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_topic, 10);
    } else {
      twist_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
    }
    use_timestamp_ = use_timestamp;  // store flag

    // Subscriptions (two topics → same callback)
    auto qos = rclcpp::QoS(rclcpp::SensorDataQoS());

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic, qos,
      std::bind(&TeleopNode::joy_callback, this, std::placeholders::_1));

    spacenav_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      spacenav_topic, qos,
      std::bind(&TeleopNode::joy_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Teleop node started. Publishing %s%s", cmd_vel_topic.c_str(), use_timestamp ? " (TwistStamped)" : " (Twist)");
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Dead‑man switch
    if (require_enable_) {
      if (enable_button_ >= static_cast<int>(msg->buttons.size()) || msg->buttons[enable_button_] == 0) {
        return;
      }
    }

    geometry_msgs::msg::Twist twist;
    if (axis_linear_  < static_cast<int>(msg->axes.size())) twist.linear.x  = msg->axes[axis_linear_]  * scale_linear_;
    if (axis_angular_ < static_cast<int>(msg->axes.size())) twist.angular.z = msg->axes[axis_angular_] * scale_angular_;

    if (use_timestamp_) {
      geometry_msgs::msg::TwistStamped ts;
      ts.header.stamp = get_clock()->now();
      ts.twist = twist;
      twist_stamped_pub_->publish(ts);
    } else {
      twist_pub_->publish(twist);
    }
  }

  // Params
  bool   use_timestamp_ {true};
  bool   require_enable_ {true};
  int    enable_button_ {5};
  int    axis_linear_ {1};
  int    axis_angular_ {0};
  double scale_linear_ {0.5};
  double scale_angular_ {0.5};

  // Interfaces
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr        twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr spacenav_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeleopNode>());
  rclcpp::shutdown();
  return 0;
}
