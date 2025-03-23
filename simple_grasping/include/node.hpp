#ifndef SIMPLE_GRASPING_NODE_HPP
#define SIMPLE_GRASPING_NODE_HPP

#include "params.hpp"
#include <simple_grasping_interfaces/action/simple_perception.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class SimpleGraspingNode : public rclcpp::Node
{
public:
  using SimplePerception = simple_grasping_interfaces::action::SimplePerception;
  using GoalHandleSimplePerception = rclcpp_action::ServerGoalHandle<SimplePerception>;

  explicit SimpleGraspingNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  NodeConfig config_;

  void sensor_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;

  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;

  rclcpp_action::Server<SimplePerception>::SharedPtr perception_action_server_;
  void execute_simple_perception(const std::shared_ptr<GoalHandleSimplePerception> goal_handle);
};

#endif // SIMPLE_GRASPING_NODE_HPP
