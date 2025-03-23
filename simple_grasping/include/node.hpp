#ifndef SIMPLE_GRASPING_NODE_HPP
#define SIMPLE_GRASPING_NODE_HPP

#include "params.hpp"
#include "filters.hpp"
#include "plane_detector.hpp"
#include "object_detector.hpp"

#include <simple_grasping_interfaces/action/simple_perception.hpp>
#include <simple_grasping_interfaces/action/simple_grasp.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

// GPD
#include <gpd/util/cloud.h>
#include <gpd/grasp_detector.h>

class SimpleGraspingNode : public rclcpp::Node
{
public:
  using SimplePerception = simple_grasping_interfaces::action::SimplePerception;
  using GoalHandleSimplePerception = rclcpp_action::ServerGoalHandle<SimplePerception>;
  using SimpleGrasp = simple_grasping_interfaces::action::SimpleGrasp;
  using GoalHandleSimpleGrasp = rclcpp_action::ServerGoalHandle<SimpleGrasp>;

  explicit SimpleGraspingNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  NodeConfig config_;

  std::unique_ptr<gpd::GraspDetector> grasp_detector_;
  std::shared_ptr<gpd::util::Cloud> gpd_cloud_;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr combined_cloud_rgb_;
  Eigen::Matrix3Xd view_points_;
  Eigen::MatrixXi camera_source_;
  

  std::vector<Plane> planes_;
  std::vector<Object> objects_;

  // Debug publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_pub_markers_;

  void sensor_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;

  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;

  rclcpp_action::Server<SimplePerception>::SharedPtr perception_action_server_;
  void execute_simple_perception(const std::shared_ptr<GoalHandleSimplePerception> goal_handle);

  rclcpp_action::Server<SimpleGrasp>::SharedPtr grasp_action_server_;
  void execute_simple_grasp(const std::shared_ptr<GoalHandleSimpleGrasp> goal_handle);

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  visualization_msgs::msg::Marker createMarkerFromOBB(const OBB &obb, const std::string &ns, int id, float r, float g, float b, float a=1.0);
};

#endif // SIMPLE_GRASPING_NODE_HPP
