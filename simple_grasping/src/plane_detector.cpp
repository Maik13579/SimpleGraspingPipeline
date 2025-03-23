#include "plane_detector.hpp"
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/transforms.h>
#include <omp.h>
#include <cmath>

namespace plane_detector {

std::vector<Plane> detect_planes(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
  const PlaneParams &params,
  int n_threads)
{
  std::vector<Plane> planes;

  // Estimate normals using OpenMP
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  ne.setNumberOfThreads(n_threads);
  ne.setInputCloud(cloud);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(params.normal_estimation_radius);
  ne.compute(*normals);

  // Filter points with nearly horizontal normals.
  pcl::PointCloud<pcl::PointXYZ>::Ptr horizontal_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    float nz = normals->points[i].normal_z;
    float angle = std::acos(std::abs(nz)) * 180.0f / static_cast<float>(M_PI);
    if (angle < params.horizontal_threshold) {
      horizontal_cloud->points.push_back(cloud->points[i]);
    }
  }
  horizontal_cloud->width = horizontal_cloud->points.size();
  horizontal_cloud->height = 1;
  horizontal_cloud->is_dense = true;

  // Cluster the horizontal points
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr cluster_tree(new pcl::search::KdTree<pcl::PointXYZ>());
  cluster_tree->setInputCloud(horizontal_cloud);
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(params.clustering_tolerance);
  ec.setMinClusterSize(params.clustering_min_cluster_size);
  ec.setMaxClusterSize(params.clustering_max_cluster_size);
  ec.setSearchMethod(cluster_tree);
  ec.setInputCloud(horizontal_cloud);
  ec.extract(cluster_indices);

  // Process each cluster in parallel using OpenMP
  std::vector<Plane> local_planes(cluster_indices.size());
  #pragma omp parallel for num_threads(n_threads)
  for (size_t i = 0; i < cluster_indices.size(); ++i) {
    // Extract the cluster points
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointIndices::Ptr indices(new pcl::PointIndices(cluster_indices[i]));
    extract.setInputCloud(horizontal_cloud);
    extract.setIndices(indices);
    extract.setNegative(false);
    extract.filter(*cluster_cloud);

    // Use RANSAC to refine the plane on the cluster
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(params.plane_thickness);
    seg.setMaxIterations(params.ransac_iterations);
    seg.setInputCloud(cluster_cloud);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty())
      continue;

    // Extract inlier points (refined plane)
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::ExtractIndices<pcl::PointXYZ> extract_inliers;
    extract_inliers.setInputCloud(cluster_cloud);
    extract_inliers.setIndices(inliers);
    extract_inliers.setNegative(false);
    extract_inliers.filter(*plane_cloud);

    if (plane_cloud->points.empty())
      continue;

    // --- Minimal OBB computation using plane projection ---

    // Get plane coefficients
    if (coefficients->values.size() < 4)
      continue;
    float a = coefficients->values[0];
    float b = coefficients->values[1];
    float c = coefficients->values[2];
    float d = coefficients->values[3];

    // Normalize plane normal and compute a point on the plane
    Eigen::Vector3f n(a, b, c);
    if(n.norm() == 0)
      continue;
    n.normalize();
    // Choose a point on the plane: p0 = -d * n (assuming coefficients normalized)
    Eigen::Vector3f p0 = -d * n;

    // Build a coordinate system for the plane.
    // u: new X axis (choose any vector orthogonal to n)
    Eigen::Vector3f u;
    if (std::abs(n.z()) < 0.99)
      u = n.cross(Eigen::Vector3f::UnitZ());
    else
      u = n.cross(Eigen::Vector3f::UnitY());
    u.normalize();
    // v: new Y axis
    Eigen::Vector3f v = n.cross(u);

    // Rotation matrix from plane coordinates to global: columns = [u, v, n]
    Eigen::Matrix3f R_plane;
    R_plane.col(0) = u;
    R_plane.col(1) = v;
    R_plane.col(2) = n;
    // Inverse transform: p_plane = R_plane.transpose()*(p - p0)

    // Project plane_cloud into plane coordinates
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud_proj(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto &pt : plane_cloud->points) {
      Eigen::Vector3f p(pt.x, pt.y, pt.z);
      Eigen::Vector3f p_plane = R_plane.transpose() * (p - p0);
      pcl::PointXYZ pt_plane;
      pt_plane.x = p_plane.x();
      pt_plane.y = p_plane.y();
      pt_plane.z = p_plane.z(); // deviation from the ideal plane
      plane_cloud_proj->points.push_back(pt_plane);
    }
    plane_cloud_proj->width = plane_cloud_proj->points.size();
    plane_cloud_proj->height = 1;
    plane_cloud_proj->is_dense = true;

    // Build 2D cloud by dropping the z coordinate
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2d(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto &pt : plane_cloud_proj->points) {
      pcl::PointXYZ pt2d;
      pt2d.x = pt.x;
      pt2d.y = pt.y;
      pt2d.z = 0;
      cloud2d->points.push_back(pt2d);
    }
    cloud2d->width = cloud2d->points.size();
    cloud2d->height = 1;
    cloud2d->is_dense = true;

    // Compute convex hull of 2D points
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud(cloud2d);
    chull.setDimension(2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr hull(new pcl::PointCloud<pcl::PointXYZ>());
    chull.reconstruct(*hull);
    if (hull->points.size() < 3)
      continue;

    // Rotating calipers: find minimal-area rectangle.
    double min_area = std::numeric_limits<double>::max();
    double best_angle = 0;
    double best_min_x = 0, best_max_x = 0, best_min_y = 0, best_max_y = 0;
    for (size_t j = 0; j < hull->points.size(); ++j) {
      size_t k = (j + 1) % hull->points.size();
      double dx = hull->points[k].x - hull->points[j].x;
      double dy = hull->points[k].y - hull->points[j].y;
      double angle = std::atan2(dy, dx);
      // Build rotation matrix for -angle
      Eigen::Matrix2f R2;
      R2 << std::cos(-angle), -std::sin(-angle),
            std::sin(-angle),  std::cos(-angle);
      double min_x = std::numeric_limits<double>::max();
      double max_x = -std::numeric_limits<double>::max();
      double min_y = std::numeric_limits<double>::max();
      double max_y = -std::numeric_limits<double>::max();
      for (const auto &pt : cloud2d->points) {
        Eigen::Vector2f p(pt.x, pt.y);
        Eigen::Vector2f pr = R2 * p;
        min_x = std::min(min_x, (double)pr.x());
        max_x = std::max(max_x, (double)pr.x());
        min_y = std::min(min_y, (double)pr.y());
        max_y = std::max(max_y, (double)pr.y());
      }
      double area = (max_x - min_x) * (max_y - min_y);
      if (area < min_area) {
        min_area = area;
        best_angle = angle;
        best_min_x = min_x;
        best_max_x = max_x;
        best_min_y = min_y;
        best_max_y = max_y;
      }
    }
    double width_box  = best_max_x - best_min_x;
    double height_box = best_max_y - best_min_y;
    double center_x_rot = (best_min_x + best_max_x) / 2.0;
    double center_y_rot = (best_min_y + best_max_y) / 2.0;
    Eigen::Vector2f center_rot(center_x_rot, center_y_rot);
    // Inverse rotation: rotate back by best_angle to get center in plane XY coordinates
    Eigen::Matrix2f R2_inv;
    R2_inv << std::cos(best_angle), -std::sin(best_angle),
              std::sin(best_angle),  std::cos(best_angle);
    Eigen::Vector2f center_xy = R2_inv * center_rot;

    // Determine z bounds from the projected cloud
    float z_min_proj = std::numeric_limits<float>::max();
    float z_max_proj = -std::numeric_limits<float>::max();
    for (const auto &pt : plane_cloud_proj->points) {
      z_min_proj = std::min(z_min_proj, pt.z);
      z_max_proj = std::max(z_max_proj, pt.z);
    }
    float z_extent = z_max_proj - z_min_proj;
    float center_z = (z_min_proj + z_max_proj) / 2.0f;

    // Minimal 3D OBB in plane coordinates
    Eigen::Vector3f obb_center_plane(center_xy.x(), center_xy.y(), center_z);
    Eigen::Vector3f obb_extents(width_box, height_box, z_extent);
    // Build 2D rotation matrix into 3D (z unchanged)
    Eigen::Matrix3f R_obb = Eigen::Matrix3f::Identity();
    R_obb.block<2,2>(0,0) = R2_inv;

    // Transform OBB from plane coordinates back to global:
    // Global center = R_plane * (OBB center) + p0
    Eigen::Vector3f global_center = R_plane * obb_center_plane + p0;
    // Global rotation = R_plane * R_obb
    Eigen::Matrix3f global_rotation = R_plane * R_obb;

    // Prepare OBB struct (store extents as half-dimensions)
    OBB obb;
    obb.center.x = global_center.x();
    obb.center.y = global_center.y();
    obb.center.z = global_center.z();
    Eigen::Vector3f half_extents = obb_extents / 2.0f;
    obb.min_pt.x = -half_extents.x();
    obb.min_pt.y = -half_extents.y();
    obb.min_pt.z = -half_extents.z();
    obb.max_pt.x = half_extents.x();
    obb.max_pt.y = half_extents.y();
    obb.max_pt.z = half_extents.z();
    obb.rotation = global_rotation;

    // Check if area (width_box * height_box) is larger than min_area threshold.
    if (width_box * height_box < params.min_area)
      continue;

    // Store plane data
    Plane detected_plane;
    detected_plane.inliers = plane_cloud;
    detected_plane.obb = obb;
    if (coefficients->values.size() >= 4) {
      detected_plane.a = coefficients->values[0];
      detected_plane.b = coefficients->values[1];
      detected_plane.c = coefficients->values[2];
      detected_plane.d = coefficients->values[3];
    }
    local_planes[i] = detected_plane;
  }

  // Collect valid planes from the local_planes vector
  for (const auto &pl : local_planes) {
    if (pl.inliers && !pl.inliers->empty()) {
      planes.push_back(pl);
    }
  }
  return planes;
}

}  // namespace plane_detector
