#include "object_detector.hpp"
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/common.h>
#include <omp.h>
#include <cmath>
#include <limits>

namespace object_detector {

std::vector<Object> detect_objects(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    std::vector<Plane> &planes,
    const ObjectParams &obj_params,
    float height_above_plane,
    float width_adjustment,
    int n_threads)
{
    std::vector<Object> objects;
    
    #pragma omp parallel for num_threads(n_threads) schedule(dynamic)
    for (size_t i = 0; i < planes.size(); ++i) {
        Plane &plane = planes[i];
    
        // AOI bounds computation
        plane.aoi.min_pt.x = plane.obb.min_pt.x - width_adjustment;
        plane.aoi.max_pt.x = plane.obb.max_pt.x + width_adjustment;
        plane.aoi.min_pt.y = plane.obb.min_pt.y - width_adjustment;
        plane.aoi.max_pt.y = plane.obb.max_pt.y + width_adjustment;
        plane.aoi.min_pt.z = - height_above_plane / 2.0f;
        plane.aoi.max_pt.z = height_above_plane / 2.0f;
    
        // Shift the AOI center upward to exclude the plane
        plane.aoi.center.x = plane.obb.center.x;
        plane.aoi.center.y = plane.obb.center.y;
        plane.aoi.center.z = plane.obb.center.z + (plane.obb.max_pt.z - plane.obb.min_pt.z) / 2.0f + height_above_plane / 2.0f + 0.001f;
        plane.aoi.rotation = plane.obb.rotation;
    
        // Transform from global to AOI coordinates (AOI center becomes the origin)
        Eigen::Affine3f transform_to_aoi = Eigen::Affine3f::Identity();
        transform_to_aoi.linear() = plane.aoi.rotation.transpose();
        transform_to_aoi.translation() = -transform_to_aoi.linear() *
          Eigen::Vector3f(plane.aoi.center.x, plane.aoi.center.y, plane.aoi.center.z);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aoi(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*cloud, *cloud_aoi, transform_to_aoi);
    
        // Crop using AOI box boundaries (centered at zero in AOI frame)
        float aoi_width  = plane.aoi.max_pt.x - plane.aoi.min_pt.x;
        float aoi_height = plane.aoi.max_pt.y - plane.aoi.min_pt.y;
        Eigen::Vector4f crop_min(-aoi_width / 2.0f, -aoi_height / 2.0f, plane.aoi.min_pt.z, 1.0f);
        Eigen::Vector4f crop_max( aoi_width / 2.0f,  aoi_height / 2.0f, plane.aoi.max_pt.z, 1.0f);
    
        pcl::CropBox<pcl::PointXYZ> crop;
        crop.setMin(crop_min);
        crop.setMax(crop_max);
        crop.setInputCloud(cloud_aoi);
        pcl::PointCloud<pcl::PointXYZ>::Ptr aoi_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        crop.filter(*aoi_cloud);
    
        if (aoi_cloud->points.empty())
            continue;
    
        // Cluster extraction from AOI cloud
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        tree->setInputCloud(aoi_cloud);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(obj_params.clustering_tolerance);
        ec.setMinClusterSize(obj_params.clustering_min_cluster_size);
        ec.setMaxClusterSize(obj_params.clustering_max_cluster_size);
        ec.setSearchMethod(tree);
        ec.setInputCloud(aoi_cloud);
        ec.extract(cluster_indices);
    
        for (const auto &indices : cluster_indices) {
            // Extract object cluster
            pcl::PointCloud<pcl::PointXYZ>::Ptr object_cluster(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::ExtractIndices<pcl::PointXYZ> extractor;
            extractor.setInputCloud(aoi_cloud);
            extractor.setIndices(pcl::make_shared<pcl::PointIndices>(indices));
            extractor.filter(*object_cluster);
    
            if (object_cluster->points.size() < 3)
                continue;
    
            // Create a 2D cloud (x,y only)
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>());
            for (const auto &pt : object_cluster->points) {
                pcl::PointXYZ pt2d;
                pt2d.x = pt.x;
                pt2d.y = pt.y;
                pt2d.z = 0;
                cloud_2d->points.push_back(pt2d);
            }

            // Compute convex hull of 2D points
            pcl::ConvexHull<pcl::PointXYZ> chull;
            chull.setInputCloud(cloud_2d);
            chull.setDimension(2);
            pcl::PointCloud<pcl::PointXYZ>::Ptr hull(new pcl::PointCloud<pcl::PointXYZ>());
            chull.reconstruct(*hull);
            if (hull->points.size() < 3)
                continue;; // skip if not enough points

            // Rotating calipers: iterate over hull edges
            double min_area = std::numeric_limits<double>::max();
            double best_angle = 0;
            double best_min_x = 0, best_max_x = 0, best_min_y = 0, best_max_y = 0;
            for (size_t j = 0; j < hull->points.size(); ++j) {
                size_t k = (j + 1) % hull->points.size();
                double dx = hull->points[k].x - hull->points[j].x;
                double dy = hull->points[k].y - hull->points[j].y;
                double angle = std::atan2(dy, dx);
                // Build rotation matrix for -angle
                Eigen::Matrix2f R;
                R << std::cos(-angle), -std::sin(-angle),
                    std::sin(-angle),  std::cos(-angle);
                double min_x = std::numeric_limits<double>::max();
                double max_x = -std::numeric_limits<double>::max();
                double min_y = std::numeric_limits<double>::max();
                double max_y = -std::numeric_limits<double>::max();
                // Rotate all 2D points
                for (const auto &pt : cloud_2d->points) {
                    Eigen::Vector2f p(pt.x, pt.y);
                    Eigen::Vector2f pr = R * p;
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

            // Compute dimensions and center in the rotated frame
            double width_box  = best_max_x - best_min_x;
            double depth_box  = best_max_y - best_min_y;
            double center_x_rot = (best_min_x + best_max_x) / 2.0;
            double center_y_rot = (best_min_y + best_max_y) / 2.0;
            Eigen::Vector2f center_rot(center_x_rot, center_y_rot);

            // Inverse rotation: rotate back by best_angle
            Eigen::Matrix2f R_inv;
            R_inv << std::cos(best_angle), -std::sin(best_angle),
                    std::sin(best_angle),  std::cos(best_angle);
            Eigen::Vector2f center_xy = R_inv * center_rot;  // center in AOI frame (XY)

            // Determine Z extent: z_min fixed to table, z_max from cluster
            float z_max = -std::numeric_limits<float>::max();
            for (const auto &pt : object_cluster->points)
            z_max = std::max(z_max, pt.z);
            float z_min = plane.aoi.min_pt.z;
            float z_extent = z_max - z_min;
            float center_z = (z_max + z_min) / 2.0f;

            // Build the box in the AOI frame:
            Eigen::Vector3f box_center_AOI(center_xy.x(), center_xy.y(), center_z);
            Eigen::Matrix3f R_box = Eigen::Matrix3f::Identity();
            R_box.block<2,2>(0,0) = R_inv;  // XY rotation from minimal rectangle
            Eigen::Vector3f box_extents(width_box, depth_box, z_extent);

            // Transform box from AOI frame to global frame
            Eigen::Vector3f global_center = plane.aoi.rotation * box_center_AOI +
                                            Eigen::Vector3f(plane.aoi.center.x, plane.aoi.center.y, plane.aoi.center.z);
            Eigen::Matrix3f global_rotation = plane.aoi.rotation * R_box;

            // Populate OBB structure
            OBB obj_obb;
            obj_obb.center.x = global_center.x();
            obj_obb.center.y = global_center.y();
            obj_obb.center.z = global_center.z();
            Eigen::Vector3f local_min = -box_extents / 2.0f;
            Eigen::Vector3f local_max =  box_extents / 2.0f;
            obj_obb.min_pt.x = local_min.x();
            obj_obb.min_pt.y = local_min.y();
            obj_obb.min_pt.z = local_min.z();
            obj_obb.max_pt.x = local_max.x();
            obj_obb.max_pt.y = local_max.y();
            obj_obb.max_pt.z = local_max.z();
            obj_obb.rotation = global_rotation;

            Object object;
            object.obb = obj_obb;
            object.plane_index = static_cast<int>(i);
            object.cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::transformPointCloud(*object_cluster, *object.cloud, transform_to_aoi.inverse());


            #pragma omp critical
            objects.push_back(object);
        }
    }
    
    return objects;
}

} // namespace object_detector
