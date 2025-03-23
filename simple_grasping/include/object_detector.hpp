#ifndef SIMPLE_GRASPING_OBJECT_DETECTOR_HPP
#define SIMPLE_GRASPING_OBJECT_DETECTOR_HPP

#include "params.hpp"
#include "plane_detector.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

// Struct to store a detected object
struct Object {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud; // in global frame
  OBB obb;                                    // OBB of the object in global frame
  int plane_index;                            // index of the plane on which the object lies
};

namespace object_detector {

// Detect objects on the given planes using the original filtered cloud.
// The area-of-interest for each plane is constructed by enlarging in z (height) and adjusting x/y by width_adjustment.
// height_above_plane: the height above the plane to search for objects (excludes the plane itself).
// width_adjustment: negative to shrink, positive to expand the plane bounding box in x and y.
// n_threads: number of threads to use with OpenMP.
std::vector<Object> detect_objects(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
  std::vector<Plane> &planes,
  const ObjectParams &obj_params,
  float height_above_plane,
  float width_adjustment,
  int n_threads);

} // namespace object_detector

#endif // SIMPLE_GRASPING_OBJECT_DETECTOR_HPP
