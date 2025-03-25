#ifndef SIMPLE_GRASPING_WORLD_MODEL_HPP
#define SIMPLE_GRASPING_WORLD_MODEL_HPP

#include <map>
#include <vector>
#include <optional>
#include <string>

#include <visualization_msgs/msg/marker.hpp>

#include "rclcpp/rclcpp.hpp"
#include "pointcloud_server_interfaces/srv/add.hpp"
#include "pointcloud_server_interfaces/srv/clear.hpp"
#include "pointcloud_server_interfaces/srv/clear_points.hpp"
#include "pointcloud_server_interfaces/srv/empty_around_point.hpp"
#include "pointcloud_server_interfaces/srv/get.hpp"
#include "pointcloud_server_interfaces/srv/save.hpp"
#include "pointcloud_server_interfaces/srv/set_grid_size.hpp"

/**
 * \brief Represents a horizontal plane detected in the scene.
 */
struct Plane 
{
  visualization_msgs::msg::Marker obb;         ///< Oriented bounding box of the plane
  float height;                                ///< Height (z value) of the plane
  std::string object_id;                       ///< Associated object ID (name or label)
};

/**
 * @brief Stores the service clients for the objects
 */
struct ServiceClients
{
  rclcpp::Client<pointcloud_server_interfaces::srv::Add>::SharedPtr add;
  rclcpp::Client<pointcloud_server_interfaces::srv::Clear>::SharedPtr clear;
  rclcpp::Client<pointcloud_server_interfaces::srv::ClearPoints>::SharedPtr clear_points;
  rclcpp::Client<pointcloud_server_interfaces::srv::EmptyAroundPoint>::SharedPtr empty_around_point;
  rclcpp::Client<pointcloud_server_interfaces::srv::Get>::SharedPtr get;
  rclcpp::Client<pointcloud_server_interfaces::srv::Save>::SharedPtr save;
  rclcpp::Client<pointcloud_server_interfaces::srv::SetGridSize>::SharedPtr set_grid_size;
};

/**
 * \brief Represents an object composed of one or more planes
 */
struct Object
{
  std::string id;               ///< Unique object identifier (semantic name)
  int32_t num_planes;           ///< Number of planes inside object
  uint64_t unique_component_id; ///< Unique component identifier
  ServiceClients clients;       ///< Service clients
};

/**
 * \brief Stores and organizes all detected planes, indexed by their height.
 */
struct PlaneDatabase
{
  std::multimap<float, Plane> planes; ///< Map of planes sorted by height

  /**
   * \brief Insert a plane into the database using its z-coordinate as key.
   * \param p The plane to insert
   */
  void insert(const Plane &p)
  {
    planes.emplace(p.obb.pose.position.z, p);
  }

  /**
   * \brief Get all planes sorted by their height (ascending).
   * \return Sorted vector of all planes
   */
  std::vector<Plane> get_sorted_planes() const
  {
    std::vector<Plane> out;
    for (const auto &kv : planes) out.push_back(kv.second);
    return out;
  }

  /**
   * \brief Query all planes whose height is within a given threshold of a reference height.
   * \param height Target z-coordinate to search near
   * \param threshold Maximum height difference
   * \return Vector of matching planes
   */
  std::vector<Plane> querry(float height, float threshold = 0.01f) const
  {
    std::vector<Plane> result;
    auto lower = planes.lower_bound(height - threshold);
    auto upper = planes.upper_bound(height + threshold);
    for (auto it = lower; it != upper; ++it)
      result.push_back(it->second);
    return result;
  }
};

/**
 * \brief Get all planes from the database that belong to a given object.
 * \param object_id The object ID to search for
 * \param db The plane database
 * \return All planes linked to the given object
 */
inline std::vector<Plane> get_planes_for_object(const std::string &object_id, const PlaneDatabase &db)
{
  std::vector<Plane> result;
  for (const auto &entry : db.planes)
  {
    if (entry.second.object_id == object_id)
      result.push_back(entry.second);
  }
  return result;
}

#endif // SIMPLE_GRASPING_WORLD_MODEL_HPP