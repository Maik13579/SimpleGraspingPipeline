#ifndef SIMPLE_GRASPING_WORLD_MODEL_HPP
#define SIMPLE_GRASPING_WORLD_MODEL_HPP

#include <map>
#include <vector>
#include <optional>
#include <string>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Vector2.h>
#include <cmath>

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
  std::string furniture_id;                    ///< Associated furniture ID (name or label)
};

/**
 * @brief Stores the service clients for the furnitures
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
 * \brief Represents an furniture composed of one or more planes
 */
struct Furniture
{
  std::string id;               ///< Unique furniture identifier (semantic name)
  int32_t num_planes;           ///< Number of planes inside furniture
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
 * \brief Get all planes from the database that belong to a given furniture.
 * \param furniture_id The furniture ID to search for
 * \param db The plane database
 * \return All planes linked to the given furniture
 */
inline std::vector<Plane> get_planes_for_furniture(const std::string &furniture_id, const PlaneDatabase &db)
{
  std::vector<Plane> result;
  for (const auto &entry : db.planes)
  {
    if (entry.second.furniture_id == furniture_id)
      result.push_back(entry.second);
  }
  return result;
}

/**
 * \brief Compute the 2D corners of a marker's oriented bounding box.
 * \param m The marker representing the OBB.
 * \return A vector of 2D points (tf2::Vector2) representing the corners.
 */
inline std::vector<tf2::Vector2> getOBBCorners(const visualization_msgs::msg::Marker &m)
{
  // Center coordinates.
  double cx = m.pose.position.x;
  double cy = m.pose.position.y;
  // Half extents from marker scale.
  double hx = m.scale.x / 2.0;
  double hy = m.scale.y / 2.0;

  // Get orientation angle (yaw) from the marker's quaternion.
  tf2::Quaternion q(
    m.pose.orientation.x,
    m.pose.orientation.y,
    m.pose.orientation.z,
    m.pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  double c = std::cos(yaw);
  double s = std::sin(yaw);

  // Corners relative to center (in local OBB coordinates).
  std::vector<tf2::Vector2> corners = {
    tf2::Vector2(-hx, -hy),
    tf2::Vector2( hx, -hy),
    tf2::Vector2( hx,  hy),
    tf2::Vector2(-hx,  hy)
  };

  // Rotate and translate corners to world (marker) coordinates.
  for (auto &corner : corners) {
    double x_new = corner.x() * c - corner.y() * s + cx;
    double y_new = corner.x() * s + corner.y() * c + cy;
    corner.setX(x_new);
    corner.setY(y_new);
  }
  return corners;
}

/**
 * \brief Project a set of 2D points onto an axis.
 * \param corners The points.
 * \param axis The axis (normalized).
 * \param min_proj Output: minimum projection value.
 * \param max_proj Output: maximum projection value.
 */
inline void projectOntoAxis(const std::vector<tf2::Vector2> &corners,
                             const tf2::Vector2 &axis, double &min_proj, double &max_proj)
{
  if(corners.empty()) return;
  min_proj = max_proj = corners[0].dot(axis);
  for (const auto &corner : corners) {
    double proj = corner.dot(axis);
    if (proj < min_proj) min_proj = proj;
    if (proj > max_proj) max_proj = proj;
  }
}

/**
 * \brief Check if two 1D projections overlap, with an optional tolerance.
 * \param minA Minimum projection for box A.
 * \param maxA Maximum projection for box A.
 * \param minB Minimum projection for box B.
 * \param maxB Maximum projection for box B.
 * \param tolerance Tolerance for overlap.
 * \return True if the projections overlap (or touch within tolerance), false otherwise.
 */
inline bool overlap(double minA, double maxA, double minB, double maxB, double tolerance = 0.0)
{
  return !(maxA < minB - tolerance || maxB < minA - tolerance);
}

/**
 * \brief Check if two oriented bounding boxes (represented by markers) intersect in the x-y plane.
 * \param m1 First marker.
 * \param m2 Second marker.
 * \param tolerance Optional tolerance.
 * \return True if the 2D projections intersect or touch, false otherwise.
 */
inline bool areOBBsIntersecting(const visualization_msgs::msg::Marker &m1,
                                const visualization_msgs::msg::Marker &m2, double tolerance = 0.0)
{
  auto corners1 = getOBBCorners(m1);
  auto corners2 = getOBBCorners(m2);

  // Collect potential separating axes from both OBBs.
  std::vector<tf2::Vector2> axes;
  for (size_t i = 0; i < corners1.size(); ++i) {
    tf2::Vector2 edge = corners1[(i + 1) % corners1.size()] - corners1[i];
    tf2::Vector2 axis(-edge.y(), edge.x());
    axis.normalize();
    axes.push_back(axis);
  }
  for (size_t i = 0; i < corners2.size(); ++i) {
    tf2::Vector2 edge = corners2[(i + 1) % corners2.size()] - corners2[i];
    tf2::Vector2 axis(-edge.y(), edge.x());
    axis.normalize();
    axes.push_back(axis);
  }

  // Check each axis for a separating gap.
  for (const auto &axis : axes) {
    double min1, max1, min2, max2;
    projectOntoAxis(corners1, axis, min1, max1);
    projectOntoAxis(corners2, axis, min2, max2);
    if (!overlap(min1, max1, min2, max2, tolerance)) {
      return false; // Found a separating axis.
    }
  }
  return true; // No separating axis found; boxes intersect.
}

/**
 * \brief Check if two planes touch (i.e. if their oriented bounding boxes intersect in the x-y plane).
 * \param p1 First plane.
 * \param p2 Second plane.
 * \param tolerance Optional tolerance for contact.
 * \return True if the planes' OBBs intersect or touch within tolerance, false otherwise.
 */
inline bool planes_touch(const Plane &p1, const Plane &p2, double tolerance = 0.0)
{
  return areOBBsIntersecting(p1.obb, p2.obb, tolerance);
}


#endif // SIMPLE_GRASPING_WORLD_MODEL_HPP