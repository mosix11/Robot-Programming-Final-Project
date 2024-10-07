#include "dmap_localization/dmap_localization_node.hpp"

DMapLocalizationNode::DMapLocalizationNode()
: Node("dmap_localization_node")
{
  // Subscribe to map, scan, and odom topics
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&DMapLocalizationNode::map_callback, this, std::placeholders::_1));
  
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10, std::bind(&DMapLocalizationNode::scan_callback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, std::bind(&DMapLocalizationNode::odom_callback, this, std::placeholders::_1));

  // TF Broadcaster for publishing map to odom transform
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  RCLCPP_INFO(this->get_logger(), "DMap Localization Node has been initialized");
}

void DMapLocalizationNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  // Store the map data
  map_ = msg;
  RCLCPP_INFO(this->get_logger(), "Map received");
  compute_distance_map();
}

void DMapLocalizationNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
  // Store the laser scan data
  last_scan_ = scan_msg;
  perform_icp();
}

void DMapLocalizationNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
  // Store the odometry data
  last_odom_ = odom_msg;
}

void DMapLocalizationNode::compute_distance_map()
{
  // Clear the distance map
  distance_map_.clear();

  // Generate the distance map from the occupancy grid
  int width = map_->info.width;
  int height = map_->info.height;
  double resolution = map_->info.resolution;

  distance_map_.resize(height, std::vector<double>(width, std::numeric_limits<double>::infinity()));

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int index = y * width + x;
      if (map_->data[index] > 80) {
        distance_map_[y][x] = 0.0; // Obstacle cells have zero distance
      }
    }
  }

  // TODO: Implement distance transform algorithm to compute distance to obstacles
}

void DMapLocalizationNode::perform_icp()
{
  if (!map_ || !last_scan_) {
    RCLCPP_WARN(this->get_logger(), "Map or Scan not available for ICP");
    return;
  }

  // TODO: Implement Iterative Closest Point (ICP) between last_scan_ and the map_ using the odometry guess
}

void DMapLocalizationNode::publish_transform()
{
  // Publish the map -> odom transform
  geometry_msgs::msg::TransformStamped transformStamped;

  transformStamped.header.stamp = this->now();
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = "odom";

  // TODO: Set the transform values based on ICP output

  tf_broadcaster_->sendTransform(transformStamped);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DMapLocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
