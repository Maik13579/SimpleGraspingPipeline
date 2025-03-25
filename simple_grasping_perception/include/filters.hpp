#ifndef SIMPLE_GRASPING_PERCEPTION_FILTERS_HPP
#define SIMPLE_GRASPING_PERCEPTION_FILTERS_HPP

#include "params.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace filters {

pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
  const FilterParams &params);

}  // namespace filters

#endif  // SIMPLE_GRASPING_PERCEPTION_FILTERS_HPP
