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
  rclcpp::TimerBase::SharedPtr timer_;
  void load_object_components(); // one shot timer callback to load components
  bool loaded_;
  /**
   * 
   * @brief Load a pointcloud server component for a given object
   * 
   * @param obj The object to load
   */
  void load_object_component(Object &obj);
  

  PlaneDatabase plane_db_;             ///< Database of all detected planes
  std::vector<Object> objects_;        ///< All detected/grouped objects

};

#endif // SIMPLE_GRASPING_WORLD_NODE_HPP
