#ifndef SIMPLE_GRASPING_NODE_HPP
#define SIMPLE_GRASPING_NODE_HPP

#include "params.hpp"
#include "filters.hpp"
#include "plane_detector.hpp"
#include "object_detector.hpp"

#include <simple_grasping_interfaces/srv/start_perception.hpp>
#include <simple_grasping_interfaces/srv/generate_grasps.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <chrono>

// GPD
#include <gpd/util/cloud.h>
#include <gpd/grasp_detector.h>

class SimpleGraspingNode : public rclcpp::Node
{
public:
  explicit SimpleGraspingNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  NodeConfig config_;
  std::unique_ptr<gpd::GraspDetector> grasp_detector_;
  gpd::util::Cloud* gpd_cloud_;

  std::vector<Plane> planes_;
  std::vector<Object> objects_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_pub_markers_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr grasp_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pre_grasp_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr retreat_grasp_pub_;

  void sensor_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;

  // Service servers
  rclcpp::Service<simple_grasping_interfaces::srv::StartPerception>::SharedPtr perception_srv_;
  void handleStartPerception(const std::shared_ptr<simple_grasping_interfaces::srv::StartPerception::Request> request,
                             std::shared_ptr<simple_grasping_interfaces::srv::StartPerception::Response> response);

  rclcpp::Service<simple_grasping_interfaces::srv::GenerateGrasps>::SharedPtr grasp_srv_;
  void handleGenerateGrasps(const std::shared_ptr<simple_grasping_interfaces::srv::GenerateGrasps::Request> request,
                            std::shared_ptr<simple_grasping_interfaces::srv::GenerateGrasps::Response> response);

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  visualization_msgs::msg::Marker createMarkerFromOBB(const OBB &obb, const std::string &ns, int id, float r, float g, float b, float a=1.0f);
  pcl::PointCloud<pcl::PointXYZ>::Ptr createPointCloudFromOBB(const OBB &obb, float resolution=0.01f);
  visualization_msgs::msg::MarkerArray createGraspMarker(
    const gpd::candidate::Hand &grasp,
    int id,
    const std::string &frame_id,
    double score,
    const Eigen::Affine3f &T_obj_inv);
};

#endif // SIMPLE_GRASPING_NODE_HPP
