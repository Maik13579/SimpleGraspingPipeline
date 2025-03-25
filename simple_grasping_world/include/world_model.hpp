#ifndef SIMPLE_GRASPING_WORLD_MODEL_HPP
#define SIMPLE_GRASPING_WORLD_MODEL_HPP

#include <map>
#include <vector>
#include <optional>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/memory.h>
#include <visualization_msgs/msg/marker.hpp>

/**
 * \brief Represents a horizontal plane detected in the scene.
 */
struct Plane 
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr inliers; ///< Points belonging to the plane
  visualization_msgs::msg::Marker obb;         ///< Oriented bounding box of the plane
  float height;                                ///< Height (z value) of the plane
  std::string object_id;                       ///< Associated object ID (name or label)
};

/**
 * \brief Represents an object composed of one or more planes and remaining points.
 */
struct Object
{
  std::string id;               ///< Unique object identifier (semantic name)
  uint64_t unique_component_id; ///< Unique component identifier
  int32_t num_planes;
  pcl::PointCloud<pcl::PointXYZ>::Ptr rest_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(); ///< Points not belonging to any plane
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