// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Yoonseok Pyo, Ryan Shim

#ifndef TURTLEBOT3_FAKE_NODE__TURTLEBOT3_FAKE_NODE_HPP_
#define TURTLEBOT3_FAKE_NODE__TURTLEBOT3_FAKE_NODE_HPP_

#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"  // added for customization
#include "tf2_msgs/msg/tf_message.hpp"
// #include "turtlebot3_msgs/msg/sensor_state.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"  // added for customization
#include "nav_msgs/msg/occupancy_grid.hpp"  // added for customization
#include "sensor_msgs/msg/laser_scan.hpp"   // added for customization
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // added for customization
#include "tf2_ros/transform_broadcaster.h"

#define LEFT 0
#define RIGHT 1

class Turtlebot3Fake : public rclcpp::Node {
 public:
  Turtlebot3Fake();
  ~Turtlebot3Fake();

 private:
  // ROS time
  rclcpp::Time last_cmd_vel_time_;
  rclcpp::Time prev_update_time_;

  // ROS timer
  rclcpp::TimerBase::SharedPtr update_timer_;
  // Added for customization
  rclcpp::TimerBase::SharedPtr laser_timer_;

  // ROS topic publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster>
      tf_broadcaster_;  // Add a broadcaster for dynamic transforms

  // Added for customization
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;

  // ROS topic subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  // Added for customization
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_pose_sub_;


  nav_msgs::msg::Odometry odom_;
  sensor_msgs::msg::JointState joint_states_;

  double wheel_speed_cmd_[2];
  double goal_linear_velocity_;
  double goal_angular_velocity_;
  double cmd_vel_timeout_;
  double last_position_[2];
  double last_velocity_[2];
  float odom_pose_[3];
  float odom_vel_[3];

  double wheel_seperation_;
  double wheel_radius_;

  // Laser parameters *Added for customization
  double laser_range_min_;
  double laser_range_max_;
  double laser_angle_increment_;
  int laser_num_beams_;
  std::string laser_frame_id_;

  // Map data *Added for customization
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  bool map_received_;
  int occupancy_threshold_;

  bool initial_pose_received_;
  geometry_msgs::msg::PoseWithCovarianceStamped map_to_odom_pose_;

  // std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

  // Function prototypes
  void init_parameters();
  void init_variables();
  void command_velocity_callback(
      const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg);
  void update_callback();
  bool update_odometry(const rclcpp::Duration& diff_time);
  void update_joint_state();
  void update_tf(geometry_msgs::msg::TransformStamped& odom_tf);
  // Added for customization
  void initial_pose_callback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  // Function to handle map updates
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  // Function to simulate laser scan
  void publish_laser_scan();

  bool will_collide(double future_x, double future_y, double robot_radius);

  double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion& quat);
};
#endif  // TURTLEBOT3_FAKE_NODE__TURTLEBOT3_FAKE_NODE_HPP_
