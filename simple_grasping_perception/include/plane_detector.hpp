#ifndef SIMPLE_GRASPING_PERCEPTION_PLANE_DETECTOR_HPP
#define SIMPLE_GRASPING_PERCEPTION_PLANE_DETECTOR_HPP

#include "params.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <Eigen/Core>

// Struct to store an oriented bounding box (OBB)
struct OBB {
  pcl::PointXYZ center;    // Center of the bounding box in the original frame
  pcl::PointXYZ min_pt;    // In PCA space (for dimensions only)
  pcl::PointXYZ max_pt;    // In PCA space (for dimensions only)
  Eigen::Matrix3f rotation;  // Rotation (from PCA) that aligns the OBB with the original frame
};

// Struct to store a detected plane
struct Plane {
  pcl::PointCloud<pcl::PointXYZ>::Ptr inliers;
  OBB obb;
  OBB aoi;
  float a, b, c, d;  // Plane equation coefficients
};

namespace plane_detector {

// Detect horizontal planes in the cloud using provided parameters and number of threads.
// It returns a list of Plane structs that satisfy the area criteria.
std::vector<Plane> detect_planes(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
  const PlaneParams &params,
  int n_threads);

}  // namespace plane_detector

#endif  // SIMPLE_GRASPING_PERCEPTION_PLANE_DETECTOR_HPP
