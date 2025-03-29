#ifndef SIMPLE_GRASPING_WORLD_TRANSFORM_HPP
#define SIMPLE_GRASPING_WORLD_TRANSFORM_HPP

#include "world_model.hpp" 
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <simple_grasping_interfaces/msg/plane.hpp>
#include <simple_grasping_interfaces/msg/object.hpp>

/**
 * @brief Transforms a plane message into a new frame.
 * 
 * It copies the marker, transforms its pose using the given transform,
 * updates the marker header, and transforms the inlier cloud.
 * 
 * @param plane The input plane message.
 * @param transformStamped The transform to apply.
 * @return simple_grasping_interfaces::msg::Plane The transformed plane message.
 */
inline simple_grasping_interfaces::msg::Plane transformPlane(
    const simple_grasping_interfaces::msg::Plane &plane,
    const geometry_msgs::msg::TransformStamped &transformStamped)
{
  simple_grasping_interfaces::msg::Plane transformed = plane;

  // Transform the marker pose.
  geometry_msgs::msg::Pose transformed_pose;
  tf2::doTransform(plane.obb.pose, transformed_pose, transformStamped);
  transformed.obb.pose = transformed_pose;
  // Update the marker header to reflect the new frame.
  transformed.obb.header = transformStamped.header;

  // Transform the inlier cloud.
  sensor_msgs::msg::PointCloud2 transformed_cloud;
  tf2::doTransform(plane.cloud, transformed_cloud, transformStamped);
  transformed.cloud = transformed_cloud;

  return transformed;
}

/**
 * @brief Transforms an object message into a new frame.
 * 
 * It copies the object's marker, transforms its pose using the given transform,
 * updates the marker header, and transforms the associated cloud.
 * 
 * @param object The input object message.
 * @param transformStamped The transform to apply.
 * @return simple_grasping_interfaces::msg::Object The transformed object message.
 */
inline simple_grasping_interfaces::msg::Object transformObject(
    const simple_grasping_interfaces::msg::Object &object,
    const geometry_msgs::msg::TransformStamped &transformStamped)
{
  simple_grasping_interfaces::msg::Object transformed = object;

  // Transform the object's marker pose.
  geometry_msgs::msg::Pose transformed_pose;
  tf2::doTransform(object.obb.pose, transformed_pose, transformStamped);
  transformed.obb.pose = transformed_pose;
  // Update marker header.
  transformed.obb.header = transformStamped.header;

  // Transform the object's cloud.
  sensor_msgs::msg::PointCloud2 transformed_cloud;
  tf2::doTransform(object.cloud, transformed_cloud, transformStamped);
  transformed.cloud = transformed_cloud;

  return transformed;
}

#endif // SIMPLE_GRASPING_WORLD_TRANSFORM_HPP
