#include "plane_detector.hpp"
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
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

    // Compute Oriented Bounding Box (OBB) aligned with the plane using PCA
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(plane_cloud);
    Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();

    // Build an affine transform to project points into the PCA space.
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    // pca.getMean() returns a 4x1 vector; use its first 3 elements as translation.
    transform.translation() = pca.getMean().head<3>();
    transform.linear() = eigen_vectors.transpose();

    // Transform the plane points into PCA space
    pcl::PointCloud<pcl::PointXYZ>::Ptr projected(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*plane_cloud, *projected, transform);

    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*projected, min_pt, max_pt);

    // Compute the center in PCA space and then transform back to original frame.
    pcl::PointXYZ center_pca;
    center_pca.x = (min_pt.x + max_pt.x) / 2.0f;
    center_pca.y = (min_pt.y + max_pt.y) / 2.0f;
    center_pca.z = (min_pt.z + max_pt.z) / 2.0f;
    // Convert to homogeneous coordinates for transformation.
    Eigen::Vector4f center_pca_h(center_pca.x, center_pca.y, center_pca.z, 1.0f);
    Eigen::Vector4f center_orig_h = transform.inverse() * center_pca_h;
    pcl::PointXYZ center_orig;
    center_orig.x = center_orig_h[0];
    center_orig.y = center_orig_h[1];
    center_orig.z = center_orig_h[2];

    // Prepare the OBB struct
    OBB obb;
    obb.min_pt = min_pt;  // dimensions (in PCA space)
    obb.max_pt = max_pt;  // dimensions (in PCA space)
    obb.rotation = eigen_vectors;
    obb.center = center_orig;

    // Check if area (width * height in PCA plane) is larger than min_area
    float width = obb.max_pt.x - obb.min_pt.x;
    float height = obb.max_pt.y - obb.min_pt.y;
    if (width * height < params.min_area)
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
