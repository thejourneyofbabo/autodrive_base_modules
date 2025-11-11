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
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <cmath>
#include <memory>
#include <string>

namespace autodrive_ackermann
{

class AutodriveToOdom : public rclcpp::Node
{
public:
  explicit AutodriveToOdom(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("autodrive_to_odom_node", options),
    x_(0.0), y_(0.0), yaw_(0.0),
    initial_imu_yaw_(0.0),
    imu_angular_velocity_alpha_(0.3),
    imu_initialized_(false),
    filtered_angular_velocity_(0.0),
    angular_velocity_filter_initialized_(false),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    // Declare parameters
    declare_parameter("vehicle_name", "roboracer_1");
    declare_parameter("odom_frame", "odom");
    declare_parameter("base_frame", "base_link");
    declare_parameter("publish_tf", true);
    declare_parameter("use_imu", true);
    declare_parameter("integration_method", "analytical");  // euler, trapezoidal, analytical
    declare_parameter("imu_angular_velocity_alpha", 0.3);
    declare_parameter("wheelbase", 0.33);  // Distance between front and rear axles (m) - default, will try TF lookup
    declare_parameter("track_width", 0.236);  // Distance between left and right wheels (m) - default
    declare_parameter("use_tf_lookup", true);  // Try to get geometry from TF

    // Get parameters
    vehicle_name_ = get_parameter("vehicle_name").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    publish_tf_ = get_parameter("publish_tf").as_bool();
    use_imu_ = get_parameter("use_imu").as_bool();
    integration_method_ = get_parameter("integration_method").as_string();
    imu_angular_velocity_alpha_ = get_parameter("imu_angular_velocity_alpha").as_double();
    wheelbase_ = get_parameter("wheelbase").as_double();
    track_width_ = get_parameter("track_width").as_double();
    bool use_tf_lookup = get_parameter("use_tf_lookup").as_bool();

    // Try to get vehicle geometry from TF
    if (use_tf_lookup) {
      lookup_vehicle_geometry();
    }

    // Validate integration method
    if (integration_method_ != "euler" && integration_method_ != "trapezoidal" &&
        integration_method_ != "analytical") {
      RCLCPP_WARN(get_logger(),
        "Invalid integration_method '%s'. Using 'analytical' as default. "
        "Valid options: 'euler', 'trapezoidal', 'analytical'",
        integration_method_.c_str());
      integration_method_ = "analytical";
    }

    // Create TF broadcaster
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    // Create publishers
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 50);

    if (use_imu_) {
      filtered_angular_velocity_pub_ = create_publisher<std_msgs::msg::Float64>(
        "/imu/filtered_angular_velocity", 10);
    }

    // Subscribe to autodrive topics
    std::string speed_topic = "/autodrive/" + vehicle_name_ + "/speed";
    std::string imu_topic = "/autodrive/" + vehicle_name_ + "/imu";
    std::string steering_topic = "/autodrive/" + vehicle_name_ + "/steering";
    std::string left_encoder_topic = "/autodrive/" + vehicle_name_ + "/left_encoder";
    std::string right_encoder_topic = "/autodrive/" + vehicle_name_ + "/right_encoder";

    speed_sub_ = create_subscription<std_msgs::msg::Float32>(
      speed_topic, 10,
      std::bind(&AutodriveToOdom::speed_callback, this, std::placeholders::_1));

    if (use_imu_) {
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, 10,
        std::bind(&AutodriveToOdom::imu_callback, this, std::placeholders::_1));
    }

    steering_sub_ = create_subscription<std_msgs::msg::Float32>(
      steering_topic, 10,
      std::bind(&AutodriveToOdom::steering_callback, this, std::placeholders::_1));

    left_encoder_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      left_encoder_topic, 10,
      std::bind(&AutodriveToOdom::left_encoder_callback, this, std::placeholders::_1));

    right_encoder_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      right_encoder_topic, 10,
      std::bind(&AutodriveToOdom::right_encoder_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Autodrive to Odom node initialized");
    RCLCPP_INFO(get_logger(), "Vehicle: %s", vehicle_name_.c_str());
    RCLCPP_INFO(get_logger(), "Using '%s' integration method", integration_method_.c_str());
    RCLCPP_INFO(get_logger(), "Use IMU: %s", use_imu_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "Wheelbase: %.3f m", wheelbase_);
    RCLCPP_INFO(get_logger(), "Track width: %.3f m", track_width_);
  }

private:
  void lookup_vehicle_geometry()
  {
    // Wait a bit for TF to be available
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    std::string base_frame = vehicle_name_;

    try {
      // Look up front wheel position
      auto front_left_tf = tf_buffer_->lookupTransform(
        base_frame, "front_left_wheel", tf2::TimePointZero);
      auto rear_left_tf = tf_buffer_->lookupTransform(
        base_frame, "rear_left_wheel", tf2::TimePointZero);
      auto front_right_tf = tf_buffer_->lookupTransform(
        base_frame, "front_right_wheel", tf2::TimePointZero);

      // Calculate wheelbase (distance between front and rear axles)
      double front_x = front_left_tf.transform.translation.x;
      double rear_x = rear_left_tf.transform.translation.x;
      wheelbase_ = front_x - rear_x;

      // Calculate track width (distance between left and right wheels)
      double left_y = front_left_tf.transform.translation.y;
      double right_y = front_right_tf.transform.translation.y;
      track_width_ = std::abs(left_y - right_y);

      RCLCPP_INFO(get_logger(), "Vehicle geometry from TF:");
      RCLCPP_INFO(get_logger(), "  Wheelbase: %.3f m (from TF)", wheelbase_);
      RCLCPP_INFO(get_logger(), "  Track width: %.3f m (from TF)", track_width_);

      // Try to get IMU offset
      try {
        auto imu_tf = tf_buffer_->lookupTransform(base_frame, "imu", tf2::TimePointZero);
        imu_offset_x_ = imu_tf.transform.translation.x;
        imu_offset_y_ = imu_tf.transform.translation.y;
        imu_offset_z_ = imu_tf.transform.translation.z;
        RCLCPP_INFO(get_logger(), "  IMU offset: [%.3f, %.3f, %.3f] m (from TF)",
                    imu_offset_x_, imu_offset_y_, imu_offset_z_);
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(get_logger(), "Could not get IMU transform: %s", ex.what());
      }

    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(get_logger(), "Could not look up vehicle geometry from TF: %s", ex.what());
      RCLCPP_WARN(get_logger(), "Using parameter values instead");
    }
  }
  void speed_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    // Check that we have IMU data if we are using it
    if (use_imu_ && !last_imu_) {
      return;
    }

    // Store current speed
    double current_speed = msg->data;
    if (std::fabs(current_speed) < 0.05) {
      current_speed = 0.0;
    }

    // Initialize last_speed_msg_ on first callback
    if (!last_speed_msg_) {
      last_speed_msg_ = msg;
      last_time_ = this->now();
      return;
    }

    // Calculate elapsed time
    auto current_time = this->now();
    auto dt = current_time - last_time_;

    // Check for abnormal dt (e.g., node restart, message dropout)
    const double MAX_DT = 1.0;  // 1 second
    if (dt.seconds() > MAX_DT || dt.seconds() < 0) {
      RCLCPP_WARN(get_logger(),
        "Abnormal dt detected: %.6f seconds. Skipping odometry update.",
        dt.seconds());
      last_speed_msg_ = msg;
      last_time_ = current_time;
      return;
    }

    // Calculate angular velocity
    double current_angular_velocity = 0.0;
    if (use_imu_) {
      // Use filtered IMU yaw rate
      current_angular_velocity = filtered_angular_velocity_;
    } else if (last_steering_) {
      // Calculate from steering angle using bicycle model
      double steering_angle = last_steering_->data;
      current_angular_velocity = current_speed * tan(steering_angle) / wheelbase_;
    }

    // Update yaw first (needed for position integration)
    double yaw_start = yaw_;
    double yaw_end = yaw_;

    if (use_imu_) {
      // Extract yaw from IMU quaternion
      tf2::Quaternion q(
        last_imu_->orientation.x,
        last_imu_->orientation.y,
        last_imu_->orientation.z,
        last_imu_->orientation.w
      );
      tf2::Matrix3x3 m(q);
      double roll, pitch, current_imu_yaw;
      m.getRPY(roll, pitch, current_imu_yaw);

      // Initialize IMU yaw offset on first IMU data
      if (!imu_initialized_) {
        initial_imu_yaw_ = current_imu_yaw;
        imu_initialized_ = true;
        RCLCPP_INFO(get_logger(), "IMU initialized with yaw offset: %.3f rad", initial_imu_yaw_);
      }

      // Apply offset to make initial yaw = 0
      yaw_end = current_imu_yaw - initial_imu_yaw_;
      yaw_ = yaw_end;
    } else {
      yaw_end = yaw_ + current_angular_velocity * dt.seconds();
      yaw_ = yaw_end;
    }

    // Propagate odometry using selected integration method
    if (integration_method_ == "trapezoidal") {
      // Trapezoidal integration: average velocity at start and end of dt
      double x_dot_start = current_speed * cos(yaw_start);
      double y_dot_start = current_speed * sin(yaw_start);
      double x_dot_end = current_speed * cos(yaw_end);
      double y_dot_end = current_speed * sin(yaw_end);

      // Use average of start and end velocities
      x_ += 0.5 * (x_dot_start + x_dot_end) * dt.seconds();
      y_ += 0.5 * (y_dot_start + y_dot_end) * dt.seconds();

    } else if (integration_method_ == "analytical") {
      // Analytical solution for Ackermann kinematics (circular arc)
      double delta_yaw = yaw_end - yaw_start;

      if (std::fabs(delta_yaw) < 1e-6) {
        // Nearly straight motion: use simple forward integration
        x_ += current_speed * cos(yaw_start) * dt.seconds();
        y_ += current_speed * sin(yaw_start) * dt.seconds();
      } else {
        // Circular arc motion: exact solution for constant curvature
        double actual_angular_velocity = delta_yaw / dt.seconds();
        double turning_radius = current_speed / actual_angular_velocity;

        // Calculate displacement in vehicle frame (arc geometry)
        double dx_vehicle = turning_radius * sin(delta_yaw);
        double dy_vehicle = turning_radius * (1.0 - cos(delta_yaw));

        // Transform to global frame using start yaw
        x_ += dx_vehicle * cos(yaw_start) - dy_vehicle * sin(yaw_start);
        y_ += dx_vehicle * sin(yaw_start) + dy_vehicle * cos(yaw_start);
      }

    } else {
      // Default: Euler integration (first-order)
      double x_dot = current_speed * cos(yaw_start);
      double y_dot = current_speed * sin(yaw_start);
      x_ += x_dot * dt.seconds();
      y_ += y_dot * dt.seconds();
    }

    // Publish TF transform
    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped.header.stamp = current_time;
      transform_stamped.header.frame_id = odom_frame_;
      transform_stamped.child_frame_id = base_frame_;

      transform_stamped.transform.translation.x = x_;
      transform_stamped.transform.translation.y = y_;
      transform_stamped.transform.translation.z = 0.0;

      transform_stamped.transform.rotation.x = 0.0;
      transform_stamped.transform.rotation.y = 0.0;
      transform_stamped.transform.rotation.z = sin(yaw_ / 2.0);
      transform_stamped.transform.rotation.w = cos(yaw_ / 2.0);

      tf_broadcaster_->sendTransform(transform_stamped);
    }

    // Publish odometry message
    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = current_time;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    // Set position
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = sin(yaw_ / 2.0);
    odom.pose.pose.orientation.w = cos(yaw_ / 2.0);

    // Set velocity
    odom.twist.twist.linear.x = current_speed;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = current_angular_velocity;

    // Set covariance (position uncertainty)
    odom.pose.covariance[0] = 0.2;   // x
    odom.pose.covariance[7] = 0.2;   // y
    odom.pose.covariance[35] = 0.4;  // yaw

    odom_pub_->publish(odom);

    // Save state for next time
    last_speed_msg_ = msg;
    last_time_ = current_time;

    RCLCPP_DEBUG(get_logger(), "Odom - x: %.3f, y: %.3f, yaw: %.3f, v: %.3f, w: %.3f",
                 x_, y_, yaw_, current_speed, current_angular_velocity);
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    last_imu_ = msg;

    // Apply low-pass filter to angular velocity
    double raw_angular_velocity = msg->angular_velocity.z;

    if (!angular_velocity_filter_initialized_) {
      // Initialize filter with first value
      filtered_angular_velocity_ = raw_angular_velocity;
      angular_velocity_filter_initialized_ = true;
    } else {
      // Apply exponential moving average (low-pass filter)
      // filtered = alpha * new + (1 - alpha) * old
      filtered_angular_velocity_ = imu_angular_velocity_alpha_ * raw_angular_velocity +
                                    (1.0 - imu_angular_velocity_alpha_) * filtered_angular_velocity_;
    }

    // Publish filtered angular velocity
    std_msgs::msg::Float64 filtered_msg;
    filtered_msg.data = filtered_angular_velocity_;
    filtered_angular_velocity_pub_->publish(filtered_msg);
  }

  void steering_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    last_steering_ = msg;
  }

  void left_encoder_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    last_left_encoder_ = msg;
    if (!msg->position.empty()) {
      RCLCPP_DEBUG(get_logger(), "Left encoder position: %.3f", msg->position[0]);
    }
  }

  void right_encoder_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    last_right_encoder_ = msg;
    if (!msg->position.empty()) {
      RCLCPP_DEBUG(get_logger(), "Right encoder position: %.3f", msg->position[0]);
    }
  }

  // Parameters
  std::string vehicle_name_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_;
  bool use_imu_;
  std::string integration_method_;
  double imu_angular_velocity_alpha_;
  double wheelbase_;
  double track_width_;
  double imu_offset_x_;
  double imu_offset_y_;
  double imu_offset_z_;

  // State variables
  double x_, y_, yaw_;
  double initial_imu_yaw_;
  bool imu_initialized_;
  double filtered_angular_velocity_;
  bool angular_velocity_filter_initialized_;
  rclcpp::Time last_time_;

  // Last messages
  std_msgs::msg::Float32::SharedPtr last_speed_msg_;
  std_msgs::msg::Float32::SharedPtr last_steering_;
  sensor_msgs::msg::JointState::SharedPtr last_left_encoder_;
  sensor_msgs::msg::JointState::SharedPtr last_right_encoder_;
  sensor_msgs::msg::Imu::SharedPtr last_imu_;

  // Publishers and subscribers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr filtered_angular_velocity_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steering_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr left_encoder_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr right_encoder_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace autodrive_ackermann

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<autodrive_ackermann::AutodriveToOdom>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
