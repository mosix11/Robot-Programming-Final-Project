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

#include "turtlebot3_fake_node/turtlebot3_fake_node.hpp"

#include <cmath>
#include <memory>
#include <string>

using namespace std::chrono_literals;

Turtlebot3Fake::Turtlebot3Fake()
    : Node("turtlebot3_fake_node"),
      map_received_(false),
      occupancy_threshold_(65),
      initial_pose_received_(false) {
  /************************************************************
  ** Initialise ROS parameters
  ************************************************************/
  init_parameters();

  /************************************************************
  ** Initialise variables
  ************************************************************/
  init_variables();

  /************************************************************
  ** Initialise ROS publishers and subscribers
  ************************************************************/
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Initialise publishers
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos);
  joint_states_pub_ =
      this->create_publisher<sensor_msgs::msg::JointState>("joint_states", qos);
  tf_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("tf", qos);


  // Initialize the broadcaster for dynamic transforms
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // Initialise subscribers
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", qos,
      std::bind(&Turtlebot3Fake::command_velocity_callback, this,
                std::placeholders::_1));

  initial_pose_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "initialpose", qos,
          std::bind(&Turtlebot3Fake::initial_pose_callback, this,
                    std::placeholders::_1));

  /************************************************************
  ** Initialize LaserScan publisher and map subscriber
  ************************************************************/
  laser_scan_pub_ =
      this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

  // Subscribe to the map topic (assuming it's being published)
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 10,
      std::bind(&Turtlebot3Fake::map_callback, this, std::placeholders::_1));

  // Initialize LaserScan timer (e.g., 10 Hz)
  laser_timer_ = this->create_wall_timer(
      100ms, std::bind(&Turtlebot3Fake::publish_laser_scan, this));

  /************************************************************
  ** initialise ROS timers
  ************************************************************/
  update_timer_ = this->create_wall_timer(
      10ms, std::bind(&Turtlebot3Fake::update_callback, this));

  RCLCPP_INFO(
      this->get_logger(),
      "Turtlebot3 fake node has been initialized with LaserScan publisher");
}

Turtlebot3Fake::~Turtlebot3Fake() {
  RCLCPP_INFO(this->get_logger(), "Turtlebot3 fake node has been terminated");
}

/********************************************************************************
** Init functions
********************************************************************************/
void Turtlebot3Fake::init_parameters() {
  // Declare parameters that may be set on this node
  this->declare_parameter<std::string>("joint_states_frame");
  this->declare_parameter<std::string>("odom_frame");
  this->declare_parameter<std::string>("base_frame");
  this->declare_parameter<double>("wheels.separation");
  this->declare_parameter<double>("wheels.radius");

  // Get parameters from yaml
  this->get_parameter_or<std::string>(
      "joint_states_frame", joint_states_.header.frame_id, "base_footprint");
  this->get_parameter_or<std::string>("odom_frame", odom_.header.frame_id,
                                      "odom");
  this->get_parameter_or<std::string>("base_frame", odom_.child_frame_id,
                                      "base_footprint");
  this->get_parameter_or<double>("wheels.separation", wheel_seperation_, 0.0);
  this->get_parameter_or<double>("wheels.radius", wheel_radius_, 0.0);

  /************************************************************
  ** Initialize LaserScan parameters
  ************************************************************/
  // Declare and get laser parameters (you can also load these from a YAML file
  // or parameters)
  this->declare_parameter<double>("laser.range_min", 0.1);
  this->declare_parameter<double>("laser.range_max", 5.0);
  this->declare_parameter<double>("laser.angle_increment",
                                  1.0 * (M_PI / 180.0));  // 1 degree in radians
  this->declare_parameter<std::string>("laser.frame_id", "base_scan");

  this->get_parameter("laser.range_min", laser_range_min_);
  this->get_parameter("laser.range_max", laser_range_max_);
  this->get_parameter("laser.angle_increment", laser_angle_increment_);
  this->get_parameter("laser.frame_id", laser_frame_id_);

  // Calculate number of beams for 360 degrees
  laser_num_beams_ = static_cast<int>((2 * M_PI) / laser_angle_increment_);
}

/************************************************************
** Init Robot's Internal State Variables
************************************************************/

void Turtlebot3Fake::init_variables() {
  // Initialise variables
  wheel_speed_cmd_[LEFT] = 0.0;
  wheel_speed_cmd_[RIGHT] = 0.0;
  goal_linear_velocity_ = 0.0;
  goal_angular_velocity_ = 0.0;
  cmd_vel_timeout_ = 1.0;
  last_position_[LEFT] = 0.0;
  last_position_[RIGHT] = 0.0;
  last_velocity_[LEFT] = 0.0;
  last_velocity_[RIGHT] = 0.0;

  // TODO(Will Son): Find more accurate covariance
  // double pcov[36] = { 0.1,   0,   0,   0,   0, 0,
  //                       0, 0.1,   0,   0,   0, 0,
  //                       0,   0, 1e6,   0,   0, 0,
  //                       0,   0,   0, 1e6,   0, 0,
  //                       0,   0,   0,   0, 1e6, 0,
  //                       0,   0,   0,   0,   0, 0.2};
  // memcpy(&(odom_.pose.covariance), pcov, sizeof(double)*36);
  // memcpy(&(odom_.twist.covariance), pcov, sizeof(double)*36);

  odom_pose_[0] = 0.0;
  odom_pose_[1] = 0.0;
  odom_pose_[2] = 0.0;
  odom_vel_[0] = 0.0;
  odom_vel_[1] = 0.0;
  odom_vel_[2] = 0.0;

  joint_states_.name.push_back("wheel_left_joint");
  joint_states_.name.push_back("wheel_right_joint");
  joint_states_.position.resize(2, 0.0);
  joint_states_.velocity.resize(2, 0.0);
  joint_states_.effort.resize(2, 0.0);

  prev_update_time_ = this->now();
  last_cmd_vel_time_ = this->now();
}

/********************************************************************************
** Callback functions for ROS subscribers
********************************************************************************/

// This function moves the robot based on the velocity comannds received on the
// `/cmd_vel` topic. The robot will collide the obstacles in the map and cannot
// move.
void Turtlebot3Fake::command_velocity_callback(
    const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg) {
  last_cmd_vel_time_ = this->now();

  // Predict the future position based on the current velocity and odom pose

  // Transform the robot's current pose from the odom frame to the map frame
  double robot_x_in_map =
      map_to_odom_pose_.pose.pose.position.x +
      odom_pose_[0] * cos(get_yaw_from_quaternion(
                          map_to_odom_pose_.pose.pose.orientation)) -
      odom_pose_[1] *
          sin(get_yaw_from_quaternion(map_to_odom_pose_.pose.pose.orientation));
  double robot_y_in_map =
      map_to_odom_pose_.pose.pose.position.y +
      odom_pose_[0] * sin(get_yaw_from_quaternion(
                          map_to_odom_pose_.pose.pose.orientation)) +
      odom_pose_[1] *
          cos(get_yaw_from_quaternion(map_to_odom_pose_.pose.pose.orientation));
  double robot_theta_in_map =
      get_yaw_from_quaternion(map_to_odom_pose_.pose.pose.orientation) +
      odom_pose_[2];

  // Predict future position in map frame
  double dt = 0.1;  // Time step for prediction, 0.1 seconds
  double future_x_in_map =
      robot_x_in_map + cmd_vel_msg->linear.x * dt * cos(robot_theta_in_map);
  double future_y_in_map =
      robot_y_in_map + cmd_vel_msg->linear.x * dt * sin(robot_theta_in_map);

  // Check if the robot will collide with an obstacle at the future position
  bool collision = will_collide(future_x_in_map, future_y_in_map,
                                0.15);  // Robot radius is 0.2 meters

  if (collision) {
    // Stop the robot if a collision is predicted
    goal_linear_velocity_ = 0.0;
    goal_angular_velocity_ = 0.0;
    RCLCPP_WARN(this->get_logger(), "Collision predicted! Stopping robot.");
  } else {
    // Apply the cmd_vel as normal if no collision is predicted
    goal_linear_velocity_ = cmd_vel_msg->linear.x;
    goal_angular_velocity_ = cmd_vel_msg->angular.z;
  }

  // Update wheel speeds
  wheel_speed_cmd_[LEFT] =
      goal_linear_velocity_ - (goal_angular_velocity_ * wheel_seperation_ / 2);
  wheel_speed_cmd_[RIGHT] =
      goal_linear_velocity_ + (goal_angular_velocity_ * wheel_seperation_ / 2);
}

// This method checks for collision based on the future position of the robot
// and the radius of the robot. Added for customization
bool Turtlebot3Fake::will_collide(double future_x, double future_y,
                                  double robot_radius) {
  if (!map_received_) {
    // If map is not received yet, assume no collision
    RCLCPP_WARN(this->get_logger(),
                "Map not received yet, skipping collision check.");
    return false;
  }

  // Get the map resolution and origin
  double resolution = map_->info.resolution;
  double origin_x = map_->info.origin.position.x;
  double origin_y = map_->info.origin.position.y;

  // Check several points around the robot's future position (sampling the
  // circle around the robot)
  int num_samples = 16;  // More samples means more accurate collision detection
  for (int i = 0; i < num_samples; ++i) {
    double angle = (2 * M_PI * i) / num_samples;
    double check_x = future_x + robot_radius * cos(angle);
    double check_y = future_y + robot_radius * sin(angle);

    // Convert the world coordinates (check_x, check_y) to map coordinates (grid
    // indices)
    int map_x = static_cast<int>((check_x - origin_x) / resolution);
    int map_y = static_cast<int>((check_y - origin_y) / resolution);

    // Check map bounds
    if (map_x < 0 || map_x >= static_cast<int>(map_->info.width) || map_y < 0 ||
        map_y >= static_cast<int>(map_->info.height)) {
      // If out of bounds, assume no collision (or optionally, you could block
      // movement)
      continue;
    }

    // Get the occupancy value at this point in the map
    int map_index = map_y * map_->info.width + map_x;
    int occupancy_value = map_->data[map_index];

    if (occupancy_value >
        occupancy_threshold_) {  // Assume occupied if occupancy >
                                 // occupancy_threshold_
      RCLCPP_INFO(this->get_logger(),
                  "Collision detected at future position (%.2f, %.2f)", check_x,
                  check_y);
      return true;  // Collision detected
    }
  }

  return false;  // No collision detected
}

// Callback function for repositioning the robot based on /initialpose topic
// Added for customization
void Turtlebot3Fake::initial_pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(),
              "Robot relocated to x: %.2f, y: %.2f, yaw: %.2f",
              msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0);

  map_to_odom_pose_ = *msg;
  if (!initial_pose_received_) {
    initial_pose_received_ = true;  // Set the flag to true
  } else {
    // Initialise variables
    wheel_speed_cmd_[LEFT] = 0.0;
    wheel_speed_cmd_[RIGHT] = 0.0;
    goal_linear_velocity_ = 0.0;
    goal_angular_velocity_ = 0.0;
    cmd_vel_timeout_ = 1.0;
    last_position_[LEFT] = 0.0;
    last_position_[RIGHT] = 0.0;
    last_velocity_[LEFT] = 0.0;
    last_velocity_[RIGHT] = 0.0;

    odom_pose_[0] = 0.0;
    odom_pose_[1] = 0.0;
    odom_pose_[2] = 0.0;
    odom_vel_[0] = 0.0;
    odom_vel_[1] = 0.0;
    odom_vel_[2] = 0.0;
    prev_update_time_ = this->now();
    last_cmd_vel_time_ = this->now();
  }

  // // Now publish the map -> odom transform using the stored initial pose
  // geometry_msgs::msg::TransformStamped map_to_odom;
  // map_to_odom.header.stamp = this->now();
  // map_to_odom.header.frame_id = "map";  // Parent frame
  // map_to_odom.child_frame_id = "odom";  // Child frame (odom)

  // // Use the pose received on `/initialpose` for the transform
  // map_to_odom.transform.translation.x =
  // map_to_odom_pose_.pose.pose.position.x; map_to_odom.transform.translation.y
  // = map_to_odom_pose_.pose.pose.position.y;
  // map_to_odom.transform.translation.z = 0.0;  // Assuming flat terrain

  // tf2::Quaternion q(
  //     map_to_odom_pose_.pose.pose.orientation.x,
  //     map_to_odom_pose_.pose.pose.orientation.y,
  //     map_to_odom_pose_.pose.pose.orientation.z,
  //     map_to_odom_pose_.pose.pose.orientation.w);
  // map_to_odom.transform.rotation.x = q.x();
  // map_to_odom.transform.rotation.y = q.y();
  // map_to_odom.transform.rotation.z = q.z();
  // map_to_odom.transform.rotation.w = q.w();

  // // Broadcast the map->odom transform
  // tf_broadcaster_->sendTransform(map_to_odom);

}

// Added for customization
void Turtlebot3Fake::map_callback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  map_ = msg;
  map_received_ = true;
  RCLCPP_INFO(this->get_logger(),
              "Map received with resolution: %.2f meters/pixel",
              map_->info.resolution);
}

/********************************************************************************
** Lazer Scan Publisher / Added for customization
********************************************************************************/

std::unordered_set<int>
    detected_beams_;  // To keep track of beams already detected
void Turtlebot3Fake::publish_laser_scan() {
  if (!map_received_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Map not received yet. Cannot publish LaserScan.");
    return;
  }
  // Only proceed if the initial pose has been received
  if (!initial_pose_received_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Waiting for initial pose...");
    return;
  }

  // Create a LaserScan message
  sensor_msgs::msg::LaserScan scan_msg;
  scan_msg.header.stamp = this->now();
  scan_msg.header.frame_id = laser_frame_id_;
  scan_msg.angle_min = -M_PI;
  scan_msg.angle_max = M_PI;
  scan_msg.angle_increment = laser_angle_increment_;
  scan_msg.time_increment = 0.0;
  scan_msg.scan_time = 0.1;  // Assuming 10 Hz
  scan_msg.range_min = laser_range_min_;
  scan_msg.range_max = laser_range_max_;

  // Initialize ranges and intensities
  scan_msg.ranges.resize(laser_num_beams_, laser_range_max_);
  scan_msg.intensities.resize(laser_num_beams_, 0.0);

  // Apply the transformation between odom and map
  double robot_x_in_map =
      map_to_odom_pose_.pose.pose.position.x +
      odom_pose_[0] * cos(get_yaw_from_quaternion(
                          map_to_odom_pose_.pose.pose.orientation)) -
      odom_pose_[1] *
          sin(get_yaw_from_quaternion(map_to_odom_pose_.pose.pose.orientation));
  double robot_y_in_map =
      map_to_odom_pose_.pose.pose.position.y +
      odom_pose_[0] * sin(get_yaw_from_quaternion(
                          map_to_odom_pose_.pose.pose.orientation)) +
      odom_pose_[1] *
          cos(get_yaw_from_quaternion(map_to_odom_pose_.pose.pose.orientation));
  double robot_theta_in_map =
      get_yaw_from_quaternion(map_to_odom_pose_.pose.pose.orientation) +
      odom_pose_[2];

  // Iterate over each beam
  for (int i = 0; i < laser_num_beams_; ++i) {
    double angle = -M_PI + i * laser_angle_increment_ +
                   robot_theta_in_map;  // Global angle in map frame

    // Ray casting parameters
    double step_size =
        map_->info.resolution;  // Step size equal to map resolution
    double current_range = laser_range_max_;
    bool hit = false;

    // Maximum number of steps
    int max_steps = static_cast<int>(laser_range_max_ / step_size);

    for (int step = 0; step < max_steps; ++step) {
      double test_x = robot_x_in_map + step_size * step * cos(angle);
      double test_y = robot_y_in_map + step_size * step * sin(angle);

      // Convert to map indices
      int map_x = static_cast<int>((test_x - map_->info.origin.position.x) /
                                   map_->info.resolution);
      int map_y = static_cast<int>((test_y - map_->info.origin.position.y) /
                                   map_->info.resolution);

      // Check map boundaries
      if (map_x < 0 || map_x >= static_cast<int>(map_->info.width) ||
          map_y < 0 || map_y >= static_cast<int>(map_->info.height)) {
        current_range = step * step_size;
        break;  // Out of map bounds
      }

      int index = map_y * map_->info.width + map_x;
      if (map_->data[index] > occupancy_threshold_) {  // Occupied if occupancy
                                                       // > occupancy_threshold_
        current_range = step * step_size;
        hit = true;
        break;  // Obstacle detected
      }
    }

    // Assign the range value
    // scan_msg.ranges[i] = std::min(current_range, laser_range_max_);
    // Optionally, set intensity based on hit
    if (hit) {
      scan_msg.ranges[i] = std::min(current_range, laser_range_max_);
      scan_msg.intensities[i] = 1.0;  // Indicate obstacle
      // Log detection if not already logged for this beam
      if (detected_beams_.find(i) == detected_beams_.end()) {
        // RCLCPP_INFO(this->get_logger(), "Obstacle detected at beam %d with
        // range %.2f meters", i, scan_msg.ranges[i]);
        double beam_angle = angle;
        beam_angle = std::atan2(std::sin(beam_angle),
                                std::cos(beam_angle));  // Normalize
        RCLCPP_INFO(
            this->get_logger(),
            "Obstacle detected at angle %.2f radians with range %.2f meters",
            beam_angle, scan_msg.ranges[i]);
        detected_beams_.insert(i);
      }
    } else {
      scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
      scan_msg.intensities[i] = 0.0;  // No obstacle

      // Remove from detected_beams_ if previously detected
      if (detected_beams_.find(i) != detected_beams_.end()) {
        // RCLCPP_INFO(this->get_logger(), "Obstacle cleared at beam %d", i);
        detected_beams_.erase(i);
      }
    }
  }

  // Publish the LaserScan message
  laser_scan_pub_->publish(scan_msg);
}

/********************************************************************************
** Update functions
********************************************************************************/
void Turtlebot3Fake::update_callback() {
  // Only proceed if the initial pose has been received
  if (!initial_pose_received_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Waiting for initial pose...");
    return;
  }
  rclcpp::Time time_now = this->now();
  rclcpp::Duration duration(time_now - prev_update_time_);
  prev_update_time_ = time_now;

  // Update odometry if a command velocity was received
  if ((time_now - last_cmd_vel_time_).nanoseconds() / 1e9 > cmd_vel_timeout_) {
    wheel_speed_cmd_[LEFT] = 0.0;
    wheel_speed_cmd_[RIGHT] = 0.0;
  }

  // Update odometry based on movement
  update_odometry(duration);

  // Publish odometry data
  odom_.header.stamp = time_now;
  odom_pub_->publish(odom_);

  // Update the joint states
  update_joint_state();
  joint_states_.header.stamp = time_now;
  joint_states_pub_->publish(joint_states_);

  // Publish the transform between odom and base_footprint
  geometry_msgs::msg::TransformStamped odom_tf;
  update_tf(odom_tf);
  tf2_msgs::msg::TFMessage odom_tf_msg;
  odom_tf_msg.transforms.push_back(odom_tf);
  tf_pub_->publish(
      odom_tf_msg);  // Publish odom->base_link/base_footprint transform

  // // Now publish the map -> odom transform using the stored initial pose
  // geometry_msgs::msg::TransformStamped map_to_odom;
  // map_to_odom.header.stamp = time_now;
  // map_to_odom.header.frame_id = "map";  // Parent frame
  // map_to_odom.child_frame_id = "odom";  // Child frame (odom)

  // // Use the pose received on `/initialpose` for the transform
  // map_to_odom.transform.translation.x =
  // map_to_odom_pose_.pose.pose.position.x; map_to_odom.transform.translation.y
  // = map_to_odom_pose_.pose.pose.position.y;
  // map_to_odom.transform.translation.z = 0.0;  // Assuming flat terrain

  // tf2::Quaternion q(
  //     map_to_odom_pose_.pose.pose.orientation.x,
  //     map_to_odom_pose_.pose.pose.orientation.y,
  //     map_to_odom_pose_.pose.pose.orientation.z,
  //     map_to_odom_pose_.pose.pose.orientation.w);
  // map_to_odom.transform.rotation.x = q.x();
  // map_to_odom.transform.rotation.y = q.y();
  // map_to_odom.transform.rotation.z = q.z();
  // map_to_odom.transform.rotation.w = q.w();

  // // Broadcast the map->odom transform
  // tf_broadcaster_->sendTransform(map_to_odom);
}

bool Turtlebot3Fake::update_odometry(const rclcpp::Duration& duration) {
  double wheel_l, wheel_r;  // rotation value of wheel [rad]
  double delta_s, delta_theta;
  double v[2], w[2];
  double step_time = duration.nanoseconds() / 1e9;  // [sec]

  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = 0.0;

  // v = translational velocity [m/s]
  // w = rotational velocity [rad/s]
  v[LEFT] = wheel_speed_cmd_[LEFT];
  w[LEFT] = v[LEFT] / wheel_radius_;  // w = v / r
  v[RIGHT] = wheel_speed_cmd_[RIGHT];
  w[RIGHT] = v[RIGHT] / wheel_radius_;

  last_velocity_[LEFT] = w[LEFT];
  last_velocity_[RIGHT] = w[RIGHT];

  wheel_l = w[LEFT] * step_time;
  wheel_r = w[RIGHT] * step_time;

  if (isnan(wheel_l)) {
    wheel_l = 0.0;
  }

  if (isnan(wheel_r)) {
    wheel_r = 0.0;
  }

  last_position_[LEFT] += wheel_l;
  last_position_[RIGHT] += wheel_r;

  delta_s = wheel_radius_ * (wheel_r + wheel_l) / 2.0;
  delta_theta = wheel_radius_ * (wheel_r - wheel_l) / wheel_seperation_;

  // compute odometric pose
  odom_pose_[0] += delta_s * cos(odom_pose_[2] + (delta_theta / 2.0));
  odom_pose_[1] += delta_s * sin(odom_pose_[2] + (delta_theta / 2.0));
  odom_pose_[2] += delta_theta;

  // compute odometric instantaneouse velocity
  odom_vel_[0] = delta_s / step_time;  // v
  odom_vel_[1] = 0.0;
  odom_vel_[2] = delta_theta / step_time;  // w

  odom_.pose.pose.position.x = odom_pose_[0];
  odom_.pose.pose.position.y = odom_pose_[1];
  odom_.pose.pose.position.z = 0;

  tf2::Quaternion q;
  q.setRPY(0, 0, odom_pose_[2]);

  odom_.pose.pose.orientation.x = q.x();
  odom_.pose.pose.orientation.y = q.y();
  odom_.pose.pose.orientation.z = q.z();
  odom_.pose.pose.orientation.w = q.w();

  // We should update the twist of the odometry
  odom_.twist.twist.linear.x = odom_vel_[0];
  odom_.twist.twist.angular.z = odom_vel_[2];

  return true;
}

void Turtlebot3Fake::update_joint_state() {
  joint_states_.position[LEFT] = last_position_[LEFT];
  joint_states_.position[RIGHT] = last_position_[RIGHT];
  joint_states_.velocity[LEFT] = last_velocity_[LEFT];
  joint_states_.velocity[RIGHT] = last_velocity_[RIGHT];
}

void Turtlebot3Fake::update_tf(geometry_msgs::msg::TransformStamped& odom_tf) {
  odom_tf.header = odom_.header;
  odom_tf.child_frame_id = odom_.child_frame_id;
  odom_tf.transform.translation.x = odom_.pose.pose.position.x;
  odom_tf.transform.translation.y = odom_.pose.pose.position.y;
  odom_tf.transform.translation.z = odom_.pose.pose.position.z;
  odom_tf.transform.rotation = odom_.pose.pose.orientation;
}

double Turtlebot3Fake::get_yaw_from_quaternion(
    const geometry_msgs::msg::Quaternion& quat) {
  tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);

  // Use tf2's Matrix3x3 to get RPY (roll, pitch, yaw)
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  return yaw;  // We only need the yaw (z-axis rotation)
}

/*****************************************************************************
** Main
*****************************************************************************/
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Turtlebot3Fake>());
  rclcpp::shutdown();

  return 0;
}
