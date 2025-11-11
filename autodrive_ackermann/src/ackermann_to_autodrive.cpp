// MIT License
//
// Copyright (c) 2025 Jisang Yun
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/float32.hpp>

#include <cmath>
#include <memory>
#include <string>

namespace autodrive_ackermann
{

class AckermannToAutodrive : public rclcpp::Node
{
public:
  explicit AckermannToAutodrive(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("ackermann_to_autodrive_node", options)
  {
    // Declare parameters with default values
    declare_parameter("speed_to_throttle_gain", 1.0);
    declare_parameter("speed_to_throttle_offset", 0.0);
    declare_parameter("steering_angle_to_servo_gain", 1.0);
    declare_parameter("steering_angle_to_servo_offset", 0.0);
    declare_parameter("max_throttle", 1.0);
    declare_parameter("min_throttle", -1.0);
    declare_parameter("max_steering", 1.0);
    declare_parameter("min_steering", -1.0);
    declare_parameter("vehicle_name", "roboracer_1");

    // Get conversion parameters
    speed_to_throttle_gain_ = get_parameter("speed_to_throttle_gain").as_double();
    speed_to_throttle_offset_ = get_parameter("speed_to_throttle_offset").as_double();
    steering_to_servo_gain_ = get_parameter("steering_angle_to_servo_gain").as_double();
    steering_to_servo_offset_ = get_parameter("steering_angle_to_servo_offset").as_double();
    max_throttle_ = get_parameter("max_throttle").as_double();
    min_throttle_ = get_parameter("min_throttle").as_double();
    max_steering_ = get_parameter("max_steering").as_double();
    min_steering_ = get_parameter("min_steering").as_double();
    vehicle_name_ = get_parameter("vehicle_name").as_string();

    // Create publishers to autodrive throttle and steering command topics
    std::string throttle_topic = "/autodrive/" + vehicle_name_ + "/throttle_command";
    std::string steering_topic = "/autodrive/" + vehicle_name_ + "/steering_command";

    throttle_pub_ = create_publisher<std_msgs::msg::Float32>(throttle_topic, 10);
    steering_pub_ = create_publisher<std_msgs::msg::Float32>(steering_topic, 10);

    // Subscribe to ackermann drive topic
    ackermann_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      "/drive", 10,
      std::bind(&AckermannToAutodrive::ackermann_cmd_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Ackermann to Autodrive node initialized");
    RCLCPP_INFO(get_logger(), "Vehicle: %s", vehicle_name_.c_str());
    RCLCPP_INFO(get_logger(), "Subscribing to: /drive");
    RCLCPP_INFO(get_logger(), "Publishing to: %s, %s", throttle_topic.c_str(), steering_topic.c_str());
  }

private:
  void ackermann_cmd_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr cmd)
  {
    // Convert speed to throttle command
    std_msgs::msg::Float32 throttle_msg;
    double throttle_value = speed_to_throttle_gain_ * cmd->drive.speed + speed_to_throttle_offset_;

    // Clamp throttle to limits
    throttle_value = std::max(min_throttle_, std::min(max_throttle_, throttle_value));
    throttle_msg.data = static_cast<float>(throttle_value);

    // Convert steering angle to steering command
    std_msgs::msg::Float32 steering_msg;
    double steering_value = steering_to_servo_gain_ * cmd->drive.steering_angle + steering_to_servo_offset_;

    // Clamp steering to limits
    steering_value = std::max(min_steering_, std::min(max_steering_, steering_value));
    steering_msg.data = static_cast<float>(steering_value);

    // Publish commands
    if (rclcpp::ok()) {
      throttle_pub_->publish(throttle_msg);
      steering_pub_->publish(steering_msg);

      RCLCPP_DEBUG(get_logger(),
                   "Published - Throttle: %.3f, Steering: %.3f (from speed: %.3f, angle: %.3f)",
                   throttle_msg.data, steering_msg.data, cmd->drive.speed, cmd->drive.steering_angle);
    }
  }

  // Conversion parameters
  double speed_to_throttle_gain_;
  double speed_to_throttle_offset_;
  double steering_to_servo_gain_;
  double steering_to_servo_offset_;
  double max_throttle_;
  double min_throttle_;
  double max_steering_;
  double min_steering_;
  std::string vehicle_name_;

  // Publishers and subscribers
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_sub_;
};

}  // namespace autodrive_ackermann

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<autodrive_ackermann::AckermannToAutodrive>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
