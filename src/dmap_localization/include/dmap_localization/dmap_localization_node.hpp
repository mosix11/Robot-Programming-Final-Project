#pragma once

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Include nanoflann for KD-tree implementations
#include <vector>

#include "nanoflann/nanoflann.hpp"

class DMapLocalizationNode : public rclcpp::Node {
 public:
  DMapLocalizationNode();
  ~DMapLocalizationNode();

 private:
  // ROS subscribers
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // ROS publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      localization_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr distance_map_pub_;

  // TF broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Map data
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  bool map_received_;
  int occupancy_threshold_;

  // Distance map
  Eigen::MatrixXf distance_map_;
  bool distance_map_computed_;

  tf2::Transform T_map_odom_;

  // Current pose estimate
  Eigen::Vector3f current_pose_;  // x, y, theta

  // Initial pose received flag
  bool initial_pose_received_;

  // Odometry data
  bool reset_odometry_pose_;
  Eigen::Vector3f current_odom_;
  Eigen::Vector3f previous_odom_;


  // Callback functions
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void initial_pose_callback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // Localization specific functions
  void compute_distance_map();
  void perform_localization(const sensor_msgs::msg::LaserScan::SharedPtr scan);
  void publish_pose();

  // Helper functions
  void logTransform(const tf2::Transform& transform);
};
