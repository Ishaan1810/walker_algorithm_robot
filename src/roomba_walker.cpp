/**
 * @file roomba_Roomba_walker.cpp
 * @author Ishaan Samir Parikh (ishaanp@umd.edu)
 * @brief This program is used to make a turtlebot move in Gazebo by avoiding
 * obstacles. This is achieved through lidar scan.
 * @version 0.1
 * @date 2023-11-29
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <chrono>
#include <functional>
#include <iostream>
#include <std_msgs/msg/string.hpp>
#include <string>
using std::placeholders::_1;
using namespace std::chrono_literals;
using LASER_SCAN = sensor_msgs::msg::LaserScan;
using TWIST = geometry_msgs::msg::Twist;

/**
 * @brief class stub that initiates the Roomba_walker node. In addition gets the laser
 * scan data and declares the methods to moce the robot accordingly.
 *
 */
class Roomba_walker : public rclcpp::Node {
 public:
  /**
   * @brief Constructing a new Roomba_walker object
   *
   */
  Roomba_walker() : Node("Roomba_walker") {
    auto callback = std::bind(&Roomba_walker::laser_scan, this, _1);
    laser_data_sub =
        this->create_subscription<LASER_SCAN>("scan", 10, callback);
    publish_velocity = this->create_publisher<TWIST>("cmd_vel", 10);
  }
 private:
  /**
   * @brief The call back function updates the message from laser scan to detect the
   * obstacle
   * @param scanData
   */
  void laser_scan(const LASER_SCAN& scanData) {
    if (scanData.header.stamp.sec == 0) {
      return;
    }

    auto laser_scan_data = scanData.ranges;
    for (int i = 330; i < 330 + 60; i++) {
      if (laser_scan_data[i % 360] < 0.8) {
        move_robot(0.0, 0.1);
      } else {
        move_robot(0.1, 0.0);
      }
    }
  }
  /**
   * @brief This function is called whenever robot needs to move forward if no
   * obstacle
   *
   * @param x_velocity
   * @param z_velocity
   */
  void move_robot(float x_velocity, float z_velocity) {
    auto velocity_msg = TWIST();
    velocity_msg.linear.x = x_velocity;
    velocity_msg.angular.z = -z_velocity;
    publish_velocity->publish(velocity_msg);
  }
  // Declaring the private variables
  rclcpp::Subscription<LASER_SCAN>::SharedPtr laser_data_sub;
  rclcpp::Publisher<TWIST>::SharedPtr publish_velocity;
  rclcpp::TimerBase::SharedPtr timer;
  LASER_SCAN last_scan_data;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Roomba_walker>());
  rclcpp::shutdown();
  return 0;
}
