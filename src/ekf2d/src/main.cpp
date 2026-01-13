#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Ekf2DNode : public rclcpp::Node
{
public:
  Ekf2DNode()
  : Node("ekf2d"),
    x_(0.0),
    y_(0.0),
    yaw_(0.0),
    vx_(0.0),
    vy_(0.0),
    latest_imu_valid_(false)
  {
    double update_rate_hz = 50.0;
    this->declare_parameter("update_rate_hz", update_rate_hz);
    this->get_parameter("update_rate_hz", update_rate_hz);

    this->declare_parameter("frame_id", std::string("odom"));
    this->declare_parameter("child_frame_id", std::string("base_link"));
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("child_frame_id", child_frame_id_);

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 50, std::bind(&Ekf2DNode::imuCallback, this, _1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    auto period = std::chrono::duration<double>(1.0 / update_rate_hz);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&Ekf2DNode::update, this));

    last_time_ = this->now();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    RCLCPP_INFO(
      this->get_logger(),
      "ekf2d node started. update_rate_hz=%.1f (translation disabled, yaw-only)",
      update_rate_hz
    );
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    latest_imu_ = *msg;
    latest_imu_valid_ = true;
  }

  void update()
  {
    auto now = this->now();
    double dt = (now - last_time_).seconds();
    if (dt <= 0.0) {
      last_time_ = now;
      return;
    }
    last_time_ = now;

    if (!latest_imu_valid_) {
      return;
    }

    // Use only gyro.z to integrate yaw (planar heading)
    double gz = latest_imu_.angular_velocity.z;

    // Integrate yaw from gyro
    yaw_ += gz * dt;
    yaw_ = std::atan2(std::sin(yaw_), std::cos(yaw_));

    // NOTE: We intentionally do NOT integrate acceleration to position.
    // x_, y_, vx_, vy_ stay ~0.0 to avoid bogus drift.

    // ---------------- Odometry message ----------------
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    double half_yaw = yaw_ * 0.5;
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = std::sin(half_yaw);
    odom.pose.pose.orientation.w = std::cos(half_yaw);

    odom.twist.twist.linear.x = vx_;
    odom.twist.twist.linear.y = vy_;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = gz;

    odom_pub_->publish(odom);

    // ---------------- TF odom -> base_link ----------------
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = now;
    tf_msg.header.frame_id = frame_id_;        // "odom"
    tf_msg.child_frame_id = child_frame_id_;   // "base_link"

    tf_msg.transform.translation.x = x_;
    tf_msg.transform.translation.y = y_;
    tf_msg.transform.translation.z = 0.0;

    tf_msg.transform.rotation = odom.pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf_msg);
  }

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // State
  double x_, y_, yaw_;
  double vx_, vy_;
  std::string frame_id_;
  std::string child_frame_id_;

  sensor_msgs::msg::Imu latest_imu_;
  bool latest_imu_valid_;
  rclcpp::Time last_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Ekf2DNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
