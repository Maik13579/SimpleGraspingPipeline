#ifndef SIMPLE_GRASPING_WORLD_MODEL_HPP
#define SIMPLE_GRASPING_WORLD_MODEL_HPP

#include <map>
#include <vector>
#include <optional>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

#include <visualization_msgs/msg/marker.hpp>

#include "rclcpp/rclcpp.hpp"
#include "pointcloud_server_interfaces/srv/add.hpp"
#include "pointcloud_server_interfaces/srv/clear.hpp"
#include "pointcloud_server_interfaces/srv/clear_points.hpp"
#include "pointcloud_server_interfaces/srv/empty_around_point.hpp"
#include "pointcloud_server_interfaces/srv/get.hpp"
#include "pointcloud_server_interfaces/srv/registration.hpp"
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
  rclcpp::Client<pointcloud_server_interfaces::srv::Registration>::SharedPtr registration;
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


// Forward declaration of planes_touch so it can be used in PlaneDatabase.
inline bool planes_touch(const Plane &p1, const Plane &p2, double tolerance = 0.0);

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
    if (p.furniture_id.empty()) //throw error
      throw std::invalid_argument("Plane must have a furniture id.");
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
  std::vector<Plane> query(float height, float threshold = 0.01f) const
  {
    std::vector<Plane> result;
    auto lower = planes.lower_bound(height - threshold);
    auto upper = planes.upper_bound(height + threshold);
    for (auto it = lower; it != upper; ++it)
      result.push_back(it->second);
    return result;
  }

 /**
   * @brief Query all planes in the database that touch the given plane.
   * @param p The reference plane.
   * @param tolerance Tolerance for touching.
   * @return Vector of planes from the database that touch p.
   */
  std::vector<Plane> query(const Plane &p, double tolerance = 0.01) const
  {
    std::vector<Plane> result;
    // Use the existing query method to narrow candidates by height.
    auto candidates = query(p.obb.pose.position.z, tolerance);
    for (const auto &cand : candidates)
    {
      if (planes_touch(p, cand, tolerance))
        result.push_back(cand);
    }
    return result;
  }

  /**
   * \brief Query all furniture ids that have a plane touching the given plane.
   * \param p The reference plane.
   * \param tolerance Tolerance for touching.
   * \return Set of furniture ids from the database that have a plane touching p.
   */
  std::set<std::string> get_touching_furniture_ids(const Plane &p, double tolerance = 0.01) const
  {
    std::set<std::string> found_furniture_ids;
    auto candidates = query(p, tolerance);
    for (const auto &cand : candidates)
    {
      if (!cand.furniture_id.empty())
        found_furniture_ids.insert(cand.furniture_id);
    }
    return found_furniture_ids;
  }

  /**
   * \brief Query all furniture ids that have a plane touching any of the given planes.
   * \param planes The list of reference planes.
   * \param tolerance Tolerance for touching.
   * \return Set of furniture ids from the database that have a plane touching any of the given planes.
   */
  std::set<std::string> get_touching_furniture_ids(const std::vector<Plane> &planes, double tolerance = 0.01) const
  {
    std::set<std::string> found_furniture_ids;
    for (const auto &p : planes)
    {
      auto ids = get_touching_furniture_ids(p, tolerance);
      for (const auto &id : ids)
      {
        if (!id.empty())
          found_furniture_ids.insert(id);
      }
    }
    return found_furniture_ids;
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
 * \return A vector of 2D points (Eigen::Vector2d) representing the corners.
 */
inline std::vector<Eigen::Vector2d> getOBBCorners(const visualization_msgs::msg::Marker &m)
{
  // Center of the marker.
  Eigen::Vector2d center(m.pose.position.x, m.pose.position.y);
  // Half-extents.
  double hx = m.scale.x / 2.0;
  double hy = m.scale.y / 2.0;

  // Define local corners.
  std::vector<Eigen::Vector2d> localCorners = {
    Eigen::Vector2d(-hx, -hy),
    Eigen::Vector2d( hx, -hy),
    Eigen::Vector2d( hx,  hy),
    Eigen::Vector2d(-hx,  hy)
  };

  // Extract yaw from the marker's quaternion.
  Eigen::Quaterniond q(m.pose.orientation.w,
                       m.pose.orientation.x,
                       m.pose.orientation.y,
                       m.pose.orientation.z);
  double yaw = std::atan2(2.0*(q.w()*q.z() + q.x()*q.y()),
                          1.0 - 2.0*(q.y()*q.y() + q.z()*q.z()));

  // Rotation matrix in 2D.
  Eigen::Matrix2d R;
  R << std::cos(yaw), -std::sin(yaw),
       std::sin(yaw),  std::cos(yaw);

  // Compute world corners.
  std::vector<Eigen::Vector2d> corners;
  for (const auto &local : localCorners) {
    corners.push_back(center + R * local);
  }
  return corners;
}

/**
 * \brief Project a set of 2D points onto an axis.
 * \param corners The points.
 * \param axis The axis (should be normalized).
 * \param min_proj Output: minimum projection value.
 * \param max_proj Output: maximum projection value.
 */
inline void projectOntoAxis(const std::vector<Eigen::Vector2d> &corners,
                                 const Eigen::Vector2d &axis,
                                 double &min_proj, double &max_proj)
{
  if (corners.empty()) return;
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

  // Collect potential separating axes from both boxes.
  std::vector<Eigen::Vector2d> axes;
  auto addAxes = [&](const std::vector<Eigen::Vector2d> &corners) {
    for (size_t i = 0; i < corners.size(); ++i) {
      Eigen::Vector2d edge = corners[(i + 1) % corners.size()] - corners[i];
      // Get perpendicular axis.
      Eigen::Vector2d axis(-edge.y(), edge.x());
      axis.normalize();
      axes.push_back(axis);
    }
  };

  addAxes(corners1);
  addAxes(corners2);

  // Check each axis for a gap.
  for (const auto &axis : axes) {
    double min1, max1, min2, max2;
    projectOntoAxis(corners1, axis, min1, max1);
    projectOntoAxis(corners2, axis, min2, max2);
    if (!overlap(min1, max1, min2, max2, tolerance))
      return false;
  }
  return true;
}

/**
 * \brief Check if two planes touch (i.e. if their oriented bounding boxes intersect in the x-y plane).
 * \param p1 First plane.
 * \param p2 Second plane.
 * \param tolerance Optional tolerance.
 * \return True if the planes' OBBs intersect or touch within tolerance, false otherwise.
 */
inline bool planes_touch(const Plane &p1, const Plane &p2, double tolerance)
{
  return areOBBsIntersecting(p1.obb, p2.obb, tolerance);
}


#endif // SIMPLE_GRASPING_WORLD_MODEL_HPP