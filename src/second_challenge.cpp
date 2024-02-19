#include "second_challenge/second_challenge.hpp"

SecondChallenge::SecondChallenge()
: Node("second_challenge")
{
  // parameter
  goal_dist_ = this->declare_parameter<double>("goal_dist", 1.0);
  velocity_ = this->declare_parameter<double>("velocity", 0.1);

  //pub, sub
  cmd_vel_pub_ = this->create_publisher<roomba_500driver_meiji::msg::RoombaCtrl>(
    "/roomba/control", rclcpp::QoS(
      1).reliable());
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::QoS(
      1).reliable(), std::bind(&SecondChallenge::laser_callback, this, std::placeholders::_1));

  //timer
  timer_ = this->create_wall_timer(500ms, std::bind(&SecondChallenge::timer_callback, this));
}

void SecondChallenge::timer_callback()
{
  process();
}

void SecondChallenge::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  laser_ = *msg;
}

void SecondChallenge::process()
{
  if (!laser_) {
    RCLCPP_WARN_STREAM(this->get_logger(), "not get data");
    return;
  }

  sensor_msgs::msg::LaserScan laser = laser_.value();
  if (!is_goal(laser)) {
    RCLCPP_INFO_STREAM(this->get_logger(), "velocity is" << velocity_);
    run(velocity_, 0.0);
  } else {
    RCLCPP_INFO_STREAM(this->get_logger(), "stop");
    run(0.0, 0.0);
  }
}

bool SecondChallenge::is_goal(sensor_msgs::msg::LaserScan & laser)
{
  const float dist = laser.ranges[laser.ranges.size() / 2];
  return dist <= goal_dist_;
}

void SecondChallenge::run(float velocity, float omega)
{
  // roombaの制御モード
  // 基本的に11（DRIVE_DIRECT）固定で良い
  cmd_vel_.mode = 11;

  // 並進速度と旋回速度を指定
  cmd_vel_.cntl.linear.x = velocity;    // 並進速度
  cmd_vel_.cntl.angular.z = omega;      // 旋回速度

  // cmd_velをpublish
  cmd_vel_pub_->publish(cmd_vel_);
}
