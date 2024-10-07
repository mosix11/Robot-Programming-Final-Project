#include <nav_msgs/msg/odometry.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

class RobotRelocator : public rclcpp::Node {
 public:
  RobotRelocator() : Node("robot_relocator") {
    subscription_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10,
        std::bind(&RobotRelocator::initialpose_callback, this,
                  std::placeholders::_1));

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  }

 private:
  void initialpose_callback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received new initial pose");

    geometry_msgs::msg::TransformStamped transformStamped;

    // Set up the transform from 'odom' to 'base_footprint'
    transformStamped.header.stamp = this->now();
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = "base_footprint";

    // Set the translation
    transformStamped.transform.translation.x = msg->pose.pose.position.x;
    transformStamped.transform.translation.y = msg->pose.pose.position.y;
    transformStamped.transform.translation.z = msg->pose.pose.position.z;

    // Set the rotation
    transformStamped.transform.rotation = msg->pose.pose.orientation;

    // Broadcast the transform
    tf_broadcaster_->sendTransform(transformStamped);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      subscription_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotRelocator>());
  rclcpp::shutdown();
  return 0;
}