#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>

#include <vector>
#include <utility>

class DMapLocalizationNode : public rclcpp::Node
{
public:
  DMapLocalizationNode();

private:
  // ROS subscribers
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // TF Broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Map and scan data
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
  nav_msgs::msg::Odometry::SharedPtr last_odom_;

  // Distance Map
  std::vector<std::vector<double>> distance_map_;

  // Methods
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);

  void compute_distance_map();
  void perform_icp();
  void publish_transform();
};


