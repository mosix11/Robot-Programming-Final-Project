
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_broadcaster.h"

class DMAPLocalizer : public rclcpp::Node {
 public:
  DMAPLocalizer() : Node("dmap_localizer_node") {
    map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10,
        std::bind(&DMAPLocalizer::MapCallback, this, std::placeholders::_1));
  }

 private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
      map_subscription_;

  void MapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(),
                "Received a map with resolution: %f, width: %d, height: %d",
                msg->info.resolution, msg->info.width, msg->info.height);

    // Access the map data
    const std::vector<int8_t>& map_data = msg->data;
    for (unsigned int y = 0; y < msg->info.height; ++y) {
      for (unsigned int x = 0; x < msg->info.width; ++x) {
        int index = x + y * msg->info.width;
        int8_t value = map_data[index];

        // -1: unknown, 0: free, 100: occupied
        if (value == -1) {
          // Handle unknown space
        } else if (value == 0) {
          // Handle free space
        } else if (value == 100) {
          // Handle occupied space
        }
      }
    }
  }
};

int main(int argc, char** argv) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create and spin the node
  auto node = std::make_shared<DMAPLocalizer>();
  rclcpp::spin(node);

  // Shutdown ROS 2
  rclcpp::shutdown();
  return 0;
}