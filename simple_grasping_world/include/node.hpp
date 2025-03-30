#ifndef SIMPLE_GRASPING_WORLD_NODE_HPP
#define SIMPLE_GRASPING_WORLD_NODE_HPP

#include "world_model.hpp"
#include "world_model_io.hpp"
#include "params.hpp"
#include "transform.hpp"
#include "utils.hpp"

#include <composition_interfaces/srv/load_node.hpp>
#include <composition_interfaces/srv/unload_node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <simple_grasping_interfaces/srv/start_perception.hpp>
#include <simple_grasping_interfaces/srv/add_frame.hpp>

#include <simple_grasping_interfaces/msg/furniture.hpp>
#include <simple_grasping_interfaces/msg/object.hpp>
#include <simple_grasping_interfaces/msg/plane.hpp>

#include <map>
#include <vector>
#include <unordered_map>
#include <optional>

class SimpleGraspingWorldNode : public rclcpp::Node
{
public:
  explicit SimpleGraspingWorldNode(const rclcpp::NodeOptions &options);

private:
  NodeConfig config_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Client<composition_interfaces::srv::LoadNode>::SharedPtr load_node_client_;
  rclcpp::Client<composition_interfaces::srv::UnloadNode>::SharedPtr unload_node_client_;
  rclcpp::Client<simple_grasping_interfaces::srv::StartPerception>::SharedPtr start_perception_client_;
  rclcpp::CallbackGroup::SharedPtr callback_group_timer_;
  rclcpp::TimerBase::SharedPtr timer_;
  void load_furnitures(); // one shot timer callback to load components
  bool loaded_;

  rclcpp::CallbackGroup::SharedPtr callback_group_add_frame_;
  rclcpp::Service<simple_grasping_interfaces::srv::AddFrame>::SharedPtr add_frame_srv_;
  void add_frame_callback(
    const std::shared_ptr<simple_grasping_interfaces::srv::AddFrame::Request> request, 
    std::shared_ptr<simple_grasping_interfaces::srv::AddFrame::Response> response);


  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  sensor_msgs::msg::PointCloud2 latest_cloud_;

  /**
   * 
   * @brief Load a furniture and create its components and service clients
   *
   * @param furniture The furniture to load
   */
  void load_furniture(Furniture &furniture);

  /**
   * 
   * @brief Load a pointcloud server component for a given furniture
   * 
   * @param furniture The furniture to load
   */
  void load_furniture_component(Furniture &furniture);

  /**
   * 
   * @brief Use the perception service
   */
  std::shared_ptr<simple_grasping_interfaces::srv::StartPerception::Response> start_perception(
    const sensor_msgs::msg::PointCloud2 &cloud,
    bool only_planes=false,
    bool sort_planes_by_height=true,
    double height_above_plane=0.3,
    double width_adjustment=-0.05,
    bool return_cloud=true);
  
  PlaneDatabase plane_db_;             ///< Database of all detected planes
  std::unordered_map<std::string, Furniture> furnitures_;  ///< All detected/grouped furnitures
};

#endif // SIMPLE_GRASPING_WORLD_NODE_HPP
