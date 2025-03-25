#ifndef SIMPLE_GRASPING_PERCEPTION_UTILS_HPP
#define SIMPLE_GRASPING_PERCEPTION_UTILS_HPP

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <string>

#include <gpd/candidate/hand_geometry.h>

// Include the header that defines OBB
#include "plane_detector.hpp"


namespace utils {


    /**
     * 
     * @brief Create a visualization_msgs::msg::Marker
     * 
     * @param ns The namespace of the marker
     * @param id The id of the marker
     * @param r The red component of the marker color
     * @param g The green component of the marker color
     * @param b The blue component of the marker color
     * @param a The alpha component of the marker color (default is 1.0f).
     * @param type The marker type (default is CUBE).
     * @return visualization_msgs::msg::Marker
     */
    inline visualization_msgs::msg::Marker createMarker(
        const std::string &ns,
        int id,
        float r, float g, float b, float a=1.0f,
        uint32_t type = visualization_msgs::msg::Marker::CUBE)
    {
        visualization_msgs::msg::Marker marker;
        marker.ns = ns;
        marker.id = id;
        marker.type = type;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = a;
        marker.pose.orientation.w = 1.0f;
        return marker;
    }

    /**
     * 
     * @brief Create a visualization_msgs::msg::Marker at a specific location
     * 
     * @param center The center of the marker
     * @param scale The scale of the marker
     * @param marker_id The id of the marker
     * @param ns The namespace of the marker
     * @param color The color of the marker
     * @return visualization_msgs::msg::Marker
     */
    inline visualization_msgs::msg::Marker createMarkerAt(
        const Eigen::Vector3f &center, 
        const Eigen::Vector3f &scale, 
        int marker_id, const std::string &ns, 
        const std::array<float, 4> &color)
    {
        visualization_msgs::msg::Marker marker = createMarker(ns, marker_id, color[0], color[1], color[2], color[3]);
        marker.pose.position.x = center.x();
        marker.pose.position.y = center.y();
        marker.pose.position.z = center.z();
        marker.scale.x = scale.x();
        marker.scale.y = scale.y();
        marker.scale.z = scale.z();
        return marker;
    };

    /**
     * @brief Create a visualization_msgs::msg::Marker from an OBB
     * 
     * @param obb The OBB to create a visualization_msgs::msg::Marker from
     * @param ns The namespace of the marker
     * @param id The id of the marker
     * @param r The red component of the marker color
     * @param g The green component of the marker color
     * @param b The blue component of the marker color
     * @param a The alpha component of the marker color
     * @return visualization_msgs::msg::Marker
     */
    inline visualization_msgs::msg::Marker createMarkerFromOBB(
        const OBB &obb,
        const std::string &ns,
        int id,
        float r, float g, float b, float a)
    {
        auto marker = createMarker(ns, id, r, g, b, a);
        // Use the min and max points from the OBB to define the scale (dimensions)
        float dx = obb.max_pt.x - obb.min_pt.x;
        float dy = obb.max_pt.y - obb.min_pt.y;
        float dz = obb.max_pt.z - obb.min_pt.z;
        marker.scale.x = dx;
        marker.scale.y = dy;
        marker.scale.z = dz;
        
        // Use the computed center in original frame for marker position.
        marker.pose.position.x = obb.center.x;
        marker.pose.position.y = obb.center.y;
        marker.pose.position.z = obb.center.z;
        
        // Convert the rotation matrix to quaternion
        Eigen::Quaternionf quat(obb.rotation);
        marker.pose.orientation.x = quat.x();
        marker.pose.orientation.y = quat.y();
        marker.pose.orientation.z = quat.z();
        marker.pose.orientation.w = quat.w();
        
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = a;    
        return marker;
    }

    /**
     * @brief Create a point cloud from an OBB
     * @param obb The OBB to create a point cloud from
     * @param resolution The resolution of the point cloud
     * @return pcl::PointCloud<pcl::PointXYZ>::Ptr
     */
    inline pcl::PointCloud<pcl::PointXYZ>::Ptr createPointCloudFromOBB(const OBB &obb, float resolution=0.01f)
    {
        // Create a new point cloud pointer
        auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

        // Compute dimensions in the local (PCA) coordinate frame.
        float dx = obb.max_pt.x - obb.min_pt.x;
        float dy = obb.max_pt.y - obb.min_pt.y;
        float dz = obb.max_pt.z - obb.min_pt.z;

        // Compute the local center (midpoint) of the bounding box.
        float mid_x = (obb.min_pt.x + obb.max_pt.x) / 2.0f;
        float mid_y = (obb.min_pt.y + obb.max_pt.y) / 2.0f;
        float mid_z = (obb.min_pt.z + obb.max_pt.z) / 2.0f;

        // Determine the number of steps in each dimension based on the desired resolution.
        int steps_x = std::max(1, static_cast<int>(dx / resolution));
        int steps_y = std::max(1, static_cast<int>(dy / resolution));
        int steps_z = std::max(1, static_cast<int>(dz / resolution));

        // Loop over the local grid and compute each point's global coordinates.
        for (int i = 0; i <= steps_x; i++) {
            float ratio_x = static_cast<float>(i) / steps_x;
            float local_x = obb.min_pt.x + ratio_x * dx;
            for (int j = 0; j <= steps_y; j++) {
                float ratio_y = static_cast<float>(j) / steps_y;
                float local_y = obb.min_pt.y + ratio_y * dy;
                for (int k = 0; k <= steps_z; k++) {
                    float ratio_z = static_cast<float>(k) / steps_z;
                    float local_z = obb.min_pt.z + ratio_z * dz;
                    Eigen::Vector3f local_point(local_x, local_y, local_z);
                    // Compute displacement from the local center.
                    Eigen::Vector3f disp = local_point - Eigen::Vector3f(mid_x, mid_y, mid_z);
                    // Transform displacement to global coordinates using the OBB rotation and add the global center.
                    Eigen::Vector3f global_point = obb.center.getVector3fMap() + obb.rotation * disp;
                    pcl::PointXYZ pt;
                    pt.x = global_point.x();
                    pt.y = global_point.y();
                    pt.z = global_point.z();
                    cloud->points.push_back(pt);
                }
            }
        }
        return cloud;
    }


/**
 * @brief Create a composite visualization of the hand in its local coordinate system.
 *
 * This function creates a set of markers representing a hand (with two fingers and a palm)
 * using only the hand geometry parameters and fixed default vectors for binormal and approach.
 * The markers are generated in the hand's local coordinate system (i.e. without any external transformation).
 *
 * @param start_marker_id The starting marker id.
 * @param hand_geometry The hand geometry parameters (depth, height, outer diameter, finger width).
 * @param binormal The binormal vector in the hand coordinate system (default is (1,0,0)).
 * @param approach The approach vector in the hand coordinate system (default is (0,0,1)).
 * @return visualization_msgs::msg::MarkerArray The marker array representing the hand.
 */
inline visualization_msgs::msg::MarkerArray createHandMarker(
    int start_marker_id,
    const gpd::candidate::HandGeometry &hand_geometry,
    const Eigen::Vector3f &binormal = Eigen::Vector3f(0.0f, 1.0f, 0.0f),
    const Eigen::Vector3f &approach = Eigen::Vector3f(1.0f, 0.0f, 0.0f))
{
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Calculate half-width using outer diameter and finger width.
    double hw = 0.5 * hand_geometry.outer_diameter_ - 0.5 * hand_geometry.finger_width_;
    
    // In the hand coordinate system (local), assume the origin is at the hand center.
    // Compute key points in the hand coordinate system.
    // Left finger (bottom) position.
    Eigen::Vector3f left_bottom = -hw * binormal;
    // Right finger (bottom) position.
    Eigen::Vector3f right_bottom = hw * binormal;
    // Left finger (top) position: shift left bottom along the approach vector.
    Eigen::Vector3f left_top = left_bottom + hand_geometry.depth_ * approach;
    // Right finger (top) position: shift right bottom along the approach vector.
    Eigen::Vector3f right_top = right_bottom + hand_geometry.depth_ * approach;
    // Compute the center of the left finger.
    Eigen::Vector3f left_center = left_bottom + 0.5f * (left_top - left_bottom);
    // Compute the center of the right finger.
    Eigen::Vector3f right_center = right_bottom + 0.5f * (right_top - right_bottom);

    Eigen::Vector3f base_center = 0.5f * (left_bottom + right_bottom) - 0.01f * approach;
    Eigen::Vector3f approach_center = base_center - 0.04f * approach;
    
    // Define dimensions for the fingers.
    Eigen::Vector3f finger_scale(hand_geometry.depth_, hand_geometry.finger_width_, hand_geometry.height_);
    // Define dimensions for the palm.
    float palm_width = (right_center - left_center).norm();
    Eigen::Vector3f base_scale(0.02f, (right_bottom - left_bottom).norm(), hand_geometry.height_);
    Eigen::Vector3f approach_scale(0.08f, hand_geometry.finger_width_, hand_geometry.height_);
    
    // Set colors for visual distinction.
    std::array<float, 4> left_color = {0.0f, 0.0f, 1.0f, 0.8f};   // Blue
    std::array<float, 4> right_color = {1.0f, 0.0f, 0.0f, 0.8f};  // Red
    std::array<float, 4> palm_color = {0.0f, 1.0f, 0.0f, 0.8f};   // Green
    std::array<float, 4> approach_color = {1.0f, 1.0f, 0.0f, 0.8f}; // Yellow

    
    // Create markers.
    marker_array.markers.push_back(createMarkerAt(left_center, finger_scale, start_marker_id,     "hand", left_color));
    marker_array.markers.push_back(createMarkerAt(right_center, finger_scale, start_marker_id + 1, "hand", right_color));
    marker_array.markers.push_back(createMarkerAt(base_center, base_scale, start_marker_id + 2, "hand", palm_color));
    marker_array.markers.push_back(createMarkerAt(approach_center, approach_scale, start_marker_id + 3, "hand", approach_color));

    return marker_array;
}

/**
 * @brief Transform a single Marker into another coordinate system.
 *
 * Applies an affine transform to both position and orientation.
 *
 * @param marker The input marker.
 * @param transform The affine transformation to apply.
 * @return visualization_msgs::msg::Marker The transformed marker.
 */
inline visualization_msgs::msg::Marker transformMarker(
    const visualization_msgs::msg::Marker &marker,
    const Eigen::Affine3f &transform)
{
    visualization_msgs::msg::Marker transformed = marker;

    Eigen::Vector3f pos(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);
    Eigen::Quaternionf quat(marker.pose.orientation.w, marker.pose.orientation.x,
                            marker.pose.orientation.y, marker.pose.orientation.z);

    Eigen::Vector3f new_pos = transform * pos;
    Eigen::Matrix3f new_rot = transform.linear() * quat.toRotationMatrix();
    Eigen::Quaternionf new_quat(new_rot);

    transformed.pose.position.x = new_pos.x();
    transformed.pose.position.y = new_pos.y();
    transformed.pose.position.z = new_pos.z();
    transformed.pose.orientation.x = new_quat.x();
    transformed.pose.orientation.y = new_quat.y();
    transformed.pose.orientation.z = new_quat.z();
    transformed.pose.orientation.w = new_quat.w();

    return transformed;
}

/**
 * @brief Transform a MarkerArray into another coordinate system.
 *
 * Applies an affine transform to each marker in the array.
 *
 * @param input_array The input MarkerArray.
 * @param transform The affine transformation to apply.
 * @return visualization_msgs::msg::MarkerArray The transformed MarkerArray.
 */
inline visualization_msgs::msg::MarkerArray transformMarkerArray(
    const visualization_msgs::msg::MarkerArray &input_array,
    const Eigen::Affine3f &transform)
{
    visualization_msgs::msg::MarkerArray transformed_array;

    for (const auto &marker : input_array.markers)
    {
        transformed_array.markers.push_back(transformMarker(marker, transform));
    }

    return transformed_array;
}


}  // namespace utils
#endif  // SIMPLE_GRASPING_PERCEPTION_UTILS_HPP
