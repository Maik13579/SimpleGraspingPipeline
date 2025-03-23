#include "node.hpp"
#include "node.hpp"
#include "params.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std::placeholders;

SimpleGraspingNode::SimpleGraspingNode(const rclcpp::NodeOptions &options)
  : rclcpp::Node("simple_grasping_node", options)
{
  // Load parameters
  load_parameters(config_, this);

  // Subscribe to input cloud topic
  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input_cloud", 10,
    std::bind(&SimpleGraspingNode::sensor_callback, this, _1)
  );

  // Create the action server for simple perception using a real callback for execution
  perception_action_server_ = rclcpp_action::create_server<SimplePerception>(
    this,
    "~/simple_perception",
    // Goal callback
    [this](const auto &, const auto &) -> rclcpp_action::GoalResponse {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    // Cancel callback
    [this](const auto &) -> rclcpp_action::CancelResponse {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    // Execute callback using a member function
    std::bind(&SimpleGraspingNode::execute_simple_perception, this, _1)
  );
}

void SimpleGraspingNode::sensor_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  latest_cloud_ = msg;
}

void SimpleGraspingNode::execute_simple_perception(const std::shared_ptr<GoalHandleSimplePerception> goal_handle)
{
  // Check if latest cloud is available
  auto result = std::make_shared<SimplePerception::Result>();
  if (!latest_cloud_) {
    result->success = false;
    result->message = "No point cloud available";
    goal_handle->abort(result);
    return;
  }

  // Convert ROS point cloud to PCL format
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg(*latest_cloud_, pcl_cloud);

  result->success = true;
  result->message = "Point cloud converted to PCL format";
  goal_handle->succeed(result);
}
