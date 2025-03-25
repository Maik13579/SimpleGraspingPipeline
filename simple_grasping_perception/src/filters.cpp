#include "filters.hpp"
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>

namespace filters {

pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
  const FilterParams &params)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr processed_cloud(new pcl::PointCloud<pcl::PointXYZ>());

  // Workspace cropping: only apply if workspace dimensions are valid
  if ( (params.workspace.x_min != params.workspace.x_max) &&
       (params.workspace.y_min != params.workspace.y_max) &&
       (params.workspace.z_min != params.workspace.z_max) )
  {
    pcl::CropBox<pcl::PointXYZ> crop;
    crop.setMin(Eigen::Vector4f(params.workspace.x_min,
                                params.workspace.y_min,
                                params.workspace.z_min, 1.0f));
    crop.setMax(Eigen::Vector4f(params.workspace.x_max,
                                params.workspace.y_max,
                                params.workspace.z_max, 1.0f));
    processed_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    crop.setInputCloud(cloud);
    crop.filter(*processed_cloud);
  }
  else {
    processed_cloud = cloud;
  }

  // Voxel downsampling: only apply if voxel_size > 0
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  if (params.voxel_size > 0.0f)
  {
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setLeafSize(params.voxel_size, params.voxel_size, params.voxel_size);
    voxel.setInputCloud(processed_cloud);
    voxel.filter(*voxel_cloud);
  }
  else {
    voxel_cloud = processed_cloud;
  }

  // Radius outlier removal: only apply if min_points > 0 and radius > 0
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  if (params.min_points > 0 && params.radius > 0.0f)
  {
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    ror.setRadiusSearch(params.radius);
    ror.setMinNeighborsInRadius(params.min_points);
    ror.setInputCloud(voxel_cloud);
    ror.filter(*filtered_cloud);
  }
  else {
    filtered_cloud = voxel_cloud;
  }

  return filtered_cloud;
}

}  // namespace filters
