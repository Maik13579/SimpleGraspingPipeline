#ifndef SIMPLE_GRASPING_WORLD_NODE_HPP
#define SIMPLE_GRASPING_WORLD_NODE_HPP

#include "world_model.hpp"
#include "world_model_io.hpp"
#include "params.hpp"

#include <composition_interfaces/srv/load_node.hpp>
#include <composition_interfaces/srv/unload_node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <simple_grasping_interfaces/srv/start_perception.hpp>

#include <map>
#include <vector>
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
  

  PlaneDatabase plane_db_;             ///< Database of all detected planes
  std::vector<Furniture> furnitures_;  ///< All detected/grouped furnitures

};

#endif // SIMPLE_GRASPING_WORLD_NODE_HPP
