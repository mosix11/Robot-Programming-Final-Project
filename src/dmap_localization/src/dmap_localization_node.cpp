#include "dmap_localization/dmap_localization_node.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <queue>

using namespace std::chrono_literals;

/********************************************************************************
** Class constructor and destructor
********************************************************************************/

DMapLocalizationNode::DMapLocalizationNode()
    : Node("dmap_localization_node"),
      map_received_(false),
      occupancy_threshold_(65),
      distance_map_computed_(false),
      initial_pose_received_(false) {
  // Initialize subscribers
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10,
      std::bind(&DMapLocalizationNode::map_callback, this,
                std::placeholders::_1));

  initial_pose_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/initialpose", 10,
          std::bind(&DMapLocalizationNode::initial_pose_callback, this,
                    std::placeholders::_1));

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&DMapLocalizationNode::scan_callback, this,
                std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&DMapLocalizationNode::odom_callback, this,
                std::placeholders::_1));

  // Initialize publisher
  localization_pose_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>(
          "/localization_pose", 10);
  distance_map_pub_ =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>("/distance_map", 10);

  // Initialize TF broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  RCLCPP_INFO(this->get_logger(), "DMap Localization Node initialized.");
}

DMapLocalizationNode::~DMapLocalizationNode() {
  RCLCPP_INFO(this->get_logger(), "DMap Localization Node shutting down.");
}

/********************************************************************************
** Map callback and distance map calculation
********************************************************************************/

void DMapLocalizationNode::map_callback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  map_ = msg;
  map_received_ = true;
  RCLCPP_INFO(this->get_logger(), "Map received.");
  compute_distance_map();
}

void DMapLocalizationNode::compute_distance_map() {
  if (!map_received_) return;

  RCLCPP_INFO(this->get_logger(), "Computing distance map...");

  auto width = map_->info.width;
  auto height = map_->info.height;

  // Initialize the distance map with infinity
  distance_map_ = Eigen::MatrixXf::Constant(
      height, width, std::numeric_limits<float>::infinity());

  // Wavefront propagation using a queue
  std::queue<std::pair<int, int>> wavefront;

  // Initialize the wavefront with obstacle cells (occupied in the occupancy
  // grid)
  for (int y = 0; y < static_cast<int>(height); ++y) {
    for (int x = 0; x < static_cast<int>(width); ++x) {
      int index = x + y * width;
      int8_t value = map_->data[index];

      if (value > occupancy_threshold_) {
        // Occupied space
        wavefront.push({x, y});
        distance_map_(y, x) = 0.0f;  // Set the distance to 0 for occupied cells
      }
    }
  }

  // Define 8-connected neighborhood
  std::vector<std::pair<int, int>> neighbors = {
      {-1, 0},  {1, 0}, {0, -1}, {0, 1},  // 4-connected neighbors
      {-1, -1}, {1, 1}, {-1, 1}, {1, -1}  // diagonal neighbors
  };

  // Wavefront propagation
  while (!wavefront.empty()) {
    auto cell = wavefront.front();
    wavefront.pop();

    int x = cell.first;
    int y = cell.second;
    float current_distance = distance_map_(y, x);

    for (auto& n : neighbors) {
      int nx = x + n.first;
      int ny = y + n.second;

      if (nx >= 0 && nx < static_cast<int>(width) && ny >= 0 &&
          ny < static_cast<int>(height)) {
        int neighbor_index = nx + ny * width;
        int8_t neighbor_value = map_->data[neighbor_index];

        // Only process free or unknown cells
        if (neighbor_value >= -1 && neighbor_value <= occupancy_threshold_) {
          float neighbor_distance = distance_map_(ny, nx);
          float new_distance = current_distance + map_->info.resolution;

          if (neighbor_distance > new_distance) {
            distance_map_(ny, nx) = new_distance;
            wavefront.push({nx, ny});
          }
        }
      }
    }
  }

  // The distances are already in meters since we added resolution at each step

  // Compute the maximum distance for normalization
  float max_distance = distance_map_.maxCoeff();

  RCLCPP_INFO(this->get_logger(),
              "Maximum distance in distance map: %.2f meters", max_distance);

  // Save the distance map to a text file
  std::ofstream distance_map_file;
  distance_map_file.open("./distance_map.txt");

  if (distance_map_file.is_open()) {
    for (int y = 0; y < distance_map_.rows(); ++y) {
      for (int x = 0; x < distance_map_.cols(); ++x) {
        distance_map_file << distance_map_(y, x);

        if (x < distance_map_.cols() - 1)
          distance_map_file
              << ", ";  // Optional: comma separation between columns
      }
      distance_map_file << "\n";  // Newline after each row
    }
    distance_map_file.close();
    RCLCPP_INFO(this->get_logger(), "Distance map saved to file.");
  } else {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to open file for saving the distance map.");
  }

  // Prepare the distance map for visualization
  nav_msgs::msg::OccupancyGrid distance_map_msg;
  distance_map_msg.header.stamp = this->now();
  distance_map_msg.header.frame_id = map_->header.frame_id;
  distance_map_msg.info = map_->info;
  distance_map_msg.data.resize(distance_map_.size());

  for (int y = 0; y < distance_map_.rows(); ++y) {
    for (int x = 0; x < distance_map_.cols(); ++x) {
      int index = x + y * distance_map_.cols();
      float distance = distance_map_(y, x);

      if (std::isinf(distance)) {
        // Unreachable cells (e.g., inside walls)
        distance_map_msg.data[index] = -1;
      } else {
        // Map distance to 0-100 for visualization
        // Invert the mapping so that cells near obstacles are darker
        int value =
            static_cast<int>((1.0f - (distance / max_distance)) * 100.0f);
        value = std::min(std::max(value, 0), 100);
        distance_map_msg.data[index] = value;
      }
    }
  }

  // Publish the distance map for visualization
  distance_map_pub_->publish(distance_map_msg);
  distance_map_computed_ = true;
  RCLCPP_INFO(this->get_logger(), "Distance map computed and published.");
}

/********************************************************************************
** Initial pose callback
********************************************************************************/

void DMapLocalizationNode::initial_pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  // Set the initial pose in the map frame
  current_pose_(0) = msg->pose.pose.position.x;
  current_pose_(1) = msg->pose.pose.position.y;

  tf2::Quaternion q_map(
      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

  tf2::Matrix3x3 m(q_map);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  current_pose_(2) = yaw;

  initial_pose_received_ = true;

  RCLCPP_INFO(this->get_logger(),
              "Initial pose received: x=%.2f, y=%.2f, yaw=%.2f",
              current_pose_(0), current_pose_(1), current_pose_(2));

  // Reset previous odometry pose
  previous_odom_ = current_odom_;

  // Compute the initial transform between map and odom frames
  tf2::Transform T_map_robot, T_robot_odom;

  // T_map_robot (from initial pose in map frame)
  T_map_robot.setOrigin(tf2::Vector3(current_pose_(0), current_pose_(1), 0.0));
  T_map_robot.setRotation(q_map);

  // T_robot_odom (from current odometry pose)
  tf2::Quaternion q_odom;
  q_odom.setRPY(0, 0, current_odom_(2));
  T_robot_odom.setOrigin(tf2::Vector3(current_odom_(0), current_odom_(1), 0.0));
  T_robot_odom.setRotation(q_odom);

  // Compute initial map->odom transform
  T_map_odom_ = T_map_robot * T_robot_odom.inverse();
}

/********************************************************************************
** Odometry callback function
********************************************************************************/

void DMapLocalizationNode::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Extract the pose from odometry
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // Update the current odometry pose
  current_odom_(0) = x;
  current_odom_(1) = y;
  current_odom_(2) = yaw;

  // Initialize previous odometry if first time
  static bool first_time = true;
  if (first_time) {
    previous_odom_ = current_odom_;
    first_time = false;
  }
}

/********************************************************************************
** Laser scanner callback
********************************************************************************/

void DMapLocalizationNode::scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  if (!distance_map_computed_ || !initial_pose_received_) return;

  perform_localization(msg);
}

/********************************************************************************
** Localization algorithm
********************************************************************************/

void DMapLocalizationNode::perform_localization(
    const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // Transform current odometry pose into map frame using T_map_odom_
  tf2::Quaternion q_odom;
  q_odom.setRPY(0, 0, current_odom_(2));
  tf2::Transform T_odom_robot;
  T_odom_robot.setOrigin(tf2::Vector3(current_odom_(0), current_odom_(1), 0.0));
  T_odom_robot.setRotation(q_odom);

  tf2::Transform T_map_robot = T_map_odom_ * T_odom_robot;

  // Extract initial guess from T_map_robot
  tf2::Vector3 position = T_map_robot.getOrigin();
  tf2::Quaternion orientation = T_map_robot.getRotation();

  double roll, pitch, yaw;
  tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  Eigen::Vector3f initial_guess;
  initial_guess(0) = position.x();
  initial_guess(1) = position.y();
  initial_guess(2) = yaw;
  // Build point cloud from laser scan
  std::vector<Eigen::Vector2f> scan_points;

  float angle = scan->angle_min;

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    float r = scan->ranges[i];
    if (std::isfinite(r) && r >= scan->range_min && r <= scan->range_max) {
      // Laser point in laser frame
      Eigen::Vector2f point_laser(r * std::cos(angle), r * std::sin(angle));
      scan_points.push_back(point_laser);
    }
    angle += scan->angle_increment;
  }

  // ICP parameters
  const int max_iterations = 10;
  const float convergence_threshold = 1e-4;

  Eigen::Vector3f pose_estimate = initial_guess;

  // ICP optimization loop
  for (int iter = 0; iter < max_iterations; ++iter) {
    // Prepare Hessian and gradient
    Eigen::MatrixXf H = Eigen::MatrixXf::Zero(3, 3);
    Eigen::VectorXf g = Eigen::VectorXf::Zero(3);

    float total_error = 0.0f;

    for (const auto& point_laser : scan_points) {
      // Transform point to world frame using current pose estimate
      float cos_theta = std::cos(pose_estimate(2));
      float sin_theta = std::sin(pose_estimate(2));

      Eigen::Vector2f point_world;
      point_world(0) = pose_estimate(0) + cos_theta * point_laser(0) -
                       sin_theta * point_laser(1);
      point_world(1) = pose_estimate(1) + sin_theta * point_laser(0) +
                       cos_theta * point_laser(1);
      // Convert world coordinates to map indices
      int map_x =
          static_cast<int>((point_world(0) - map_->info.origin.position.x) /
                           map_->info.resolution);
      int map_y =
          static_cast<int>((point_world(1) - map_->info.origin.position.y) /
                           map_->info.resolution);

      if (map_x >= 1 && map_x < static_cast<int>(map_->info.width) - 1 &&
          map_y >= 1 && map_y < static_cast<int>(map_->info.height) - 1) {
        // Get the distance value
        float distance = distance_map_(map_y, map_x);

        // Compute the gradient numerically
        float grad_x = (distance_map_(map_y, map_x + 1) -
                        distance_map_(map_y, map_x - 1)) /
                       (2 * map_->info.resolution);
        float grad_y = (distance_map_(map_y + 1, map_x) -
                        distance_map_(map_y - 1, map_x)) /
                       (2 * map_->info.resolution);

        Eigen::Vector2f gradient(grad_x, grad_y);

        // Compute the error
        total_error += distance * distance;

        // Compute the Jacobian
        Eigen::Matrix<float, 1, 3> J;

        // Partial derivatives w.r.t x, y, theta
        J(0, 0) = grad_x;
        J(0, 1) = grad_y;
        J(0, 2) =
            grad_x *
                (-sin_theta * point_laser(0) - cos_theta * point_laser(1)) +
            grad_y * (cos_theta * point_laser(0) - sin_theta * point_laser(1));

        // Accumulate Hessian and gradient
        H += J.transpose() * J;
        g += J.transpose() * distance;
      }
    }
    // Solve for the pose update
    Eigen::Vector3f delta_pose = -H.ldlt().solve(g);
    // Update the pose estimate
    pose_estimate += delta_pose;
    // Normalize angle
    pose_estimate(2) =
        std::atan2(std::sin(pose_estimate(2)), std::cos(pose_estimate(2)));
    // Check for convergence
    if (delta_pose.norm() < convergence_threshold) {
      break;
    }
  }

  // Update the current pose estimate
  current_pose_ = pose_estimate;

  // Update previous odometry pose
  previous_odom_ = current_odom_;

  // Update T_map_odom_
  tf2::Transform T_map_robot_new;
  tf2::Quaternion q_map_robot_new;
  q_map_robot_new.setRPY(0, 0, current_pose_(2));
  T_map_robot_new.setOrigin(
      tf2::Vector3(current_pose_(0), current_pose_(1), 0.0));
  T_map_robot_new.setRotation(q_map_robot_new);

  // T_odom_robot (from current odometry pose)
  T_odom_robot.setOrigin(tf2::Vector3(current_odom_(0), current_odom_(1), 0.0));
  T_odom_robot.setRotation(q_odom);

  // Update map->odom transform
  T_map_odom_ = T_map_robot_new * T_odom_robot.inverse();

  // Publish the pose
  publish_pose();
}

/********************************************************************************
** Publishsing the localization estimated values
********************************************************************************/

void DMapLocalizationNode::publish_pose() {
  // Publish localization_pose for visualization stuff
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = this->now();
  pose_msg.header.frame_id = "map";

  pose_msg.pose.position.x = current_pose_(0);
  pose_msg.pose.position.y = current_pose_(1);
  pose_msg.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, current_pose_(2));
  pose_msg.pose.orientation = tf2::toMsg(q);

  localization_pose_pub_->publish(pose_msg);

  // Publish the estimated map to odom transformation on the
  // Use T_map_odom_ to send the TransformBroadcaster
  geometry_msgs::msg::TransformStamped transformStamped;

  transformStamped.header.stamp = this->now();
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = "odom";

  transformStamped.transform.translation.x = T_map_odom_.getOrigin().x();
  transformStamped.transform.translation.y = T_map_odom_.getOrigin().y();
  transformStamped.transform.translation.z = T_map_odom_.getOrigin().z();

  tf2::Quaternion q_map_odom = T_map_odom_.getRotation();

  transformStamped.transform.rotation.x = q_map_odom.x();
  transformStamped.transform.rotation.y = q_map_odom.y();
  transformStamped.transform.rotation.z = q_map_odom.z();
  transformStamped.transform.rotation.w = q_map_odom.w();

  tf_broadcaster_->sendTransform(transformStamped);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DMapLocalizationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
