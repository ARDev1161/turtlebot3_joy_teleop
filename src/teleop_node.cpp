#include <memory>
#include <stdexcept>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class TeleopNode : public rclcpp::Node
{
public:
  TeleopNode()
  : Node("turtlebot3_joy_teleop")
  {
    // Declare parameters
    declare_parameter("joy_topic",        "joy");              // вход 1
    declare_parameter("spacenav_topic",   "spacenav/joy");     // вход 2 (можно "")
    declare_parameter("cmd_vel_topic",    "cmd_vel");          // выход
    declare_parameter("use_timestamp",    true);              // Twist vs TwistStamped

    // Параметры джойстика
    declare_parameter("joy_enable_button", 5);
    declare_parameter("joy_axis_linear", 1);
    declare_parameter("joy_axis_angular", 0);
    // Параметры SpaceNav
    declare_parameter("spnav_enable_button", 0);
    declare_parameter("spnav_axis_linear", 2);
    declare_parameter("spnav_axis_angular", 4);
    declare_parameter("scale_linear", 0.5);  // макс. скорость, м/с
    declare_parameter("scale_angular", 0.5); // макс. угл. скорость, рад/с
    declare_parameter("require_enable", true);// нужно ли держать кнопку

    // Get parameters
    joy_topic_       = get_parameter("joy_topic").as_string();
    spacenav_topic_  = get_parameter("spacenav_topic").as_string();
    cmd_vel_topic_   = get_parameter("cmd_vel_topic").as_string();
    use_timestamp_   = get_parameter("use_timestamp").as_bool();

    joy_enable_button_  = this->get_parameter("joy_enable_button").as_int();
    joy_axis_linear_    = this->get_parameter("joy_axis_linear").as_int();
    joy_axis_angular_   = this->get_parameter("joy_axis_angular").as_int();
    spnav_enable_button_= this->get_parameter("spnav_enable_button").as_int();
    spnav_axis_linear_  = this->get_parameter("spnav_axis_linear").as_int();
    spnav_axis_angular_ = this->get_parameter("spnav_axis_angular").as_int();
    scale_linear_  = this->get_parameter("scale_linear").as_double();
    scale_angular_ = this->get_parameter("scale_angular").as_double();
    require_enable_= this->get_parameter("require_enable").as_bool();

    // Publisher for /cmd_vel
    if (use_timestamp_) {
      twist_stamped_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
                              cmd_vel_topic_, 10);
    } else {
      twist_pub_ = create_publisher<geometry_msgs::msg::Twist>(
                              cmd_vel_topic_, 10);
    }

    // Subscribers for joystick and spacenav
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_, 10,
      std::bind(&TeleopNode::joy_callback, this, std::placeholders::_1, false)
    );
    spacenav_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      spacenav_topic_, 10,
      std::bind(&TeleopNode::joy_callback, this, std::placeholders::_1, true)
    );

    RCLCPP_INFO(this->get_logger(),
                "Joystick teleop C++ node started (out: %s, stamped: %s)",
                cmd_vel_topic_.c_str(),
                use_timestamp_ ? "true" : "false");
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg, bool from_spnav)
  {
    int enable_button  = from_spnav ? spnav_enable_button_  : joy_enable_button_;
    int axis_linear    = from_spnav ? spnav_axis_linear_    : joy_axis_linear_;
    int axis_angular   = from_spnav ? spnav_axis_angular_   : joy_axis_angular_;

    // If enable button required but not pressed, skip
    if (require_enable_) {
      if (enable_button >= static_cast<int>(msg->buttons.size()) ||
          msg->buttons[enable_button] == 0)
        return;
    }

    geometry_msgs::msg::Twist twist;
    if (axis_linear < static_cast<int>(msg->axes.size())) {
      twist.linear.x = msg->axes[axis_linear] * scale_linear_;
    }
    if (axis_angular < static_cast<int>(msg->axes.size())) {
      twist.angular.z = msg->axes[axis_angular] * scale_angular_;
    }

    if (use_timestamp_) {
      geometry_msgs::msg::TwistStamped ts;
      ts.header.stamp = this->get_clock()->now();
      ts.twist = twist;
      twist_stamped_pub_->publish(ts);
    } else {
      twist_pub_->publish(twist);
    }

    RCLCPP_DEBUG(get_logger(),
            "callback: axes[0]=%.2f axes[1]=%.2f  publish...",
            msg->axes.size() ? msg->axes[0] : 0.0,
            msg->axes.size() > 1 ? msg->axes[1] : 0.0);
  }

  // Parameters
  std::string joy_topic_, spacenav_topic_, cmd_vel_topic_;
  bool   use_timestamp_, require_enable_;
  int    joy_enable_button_,  joy_axis_linear_,  joy_axis_angular_;
  int    spnav_enable_button_, spnav_axis_linear_, spnav_axis_angular_;
  double scale_linear_, scale_angular_;

  // ROS interfaces
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr        twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_pub_;

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
