#ifndef SECOND_CHALLENGE_HPP
#define SECOND_CHALLENGE_HPP

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <optional>
#include <functional>
#include <chrono>

#include <sensor_msgs/msg/laser_scan.hpp>

#include "roomba_500driver_meiji/msg/roomba_ctrl.hpp"

using namespace std::chrono_literals;

class SecondChallenge : public rclcpp::Node
{
public:
  SecondChallenge();

  // コールバック関数
  void timer_callback();
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);


  // 関数
  void process();
  bool is_goal(sensor_msgs::msg::LaserScan & laser);
  void run(float velocity, float omega);

  // 変数
  double goal_dist_ = 1;
  double velocity_ = 0.1;
  std::optional<sensor_msgs::msg::LaserScan> laser_;
  roomba_500driver_meiji::msg::RoombaCtrl cmd_vel_;

  rclcpp::Publisher<roomba_500driver_meiji::msg::RoombaCtrl>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // SECOND_CHALLENGE_HPP
