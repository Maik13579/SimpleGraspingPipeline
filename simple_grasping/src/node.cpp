#include "node.hpp"
#include "params.hpp"
#include "filters.hpp"
#include "plane_detector.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

using namespace std::placeholders;

SimpleGraspingNode::SimpleGraspingNode(const rclcpp::NodeOptions &options)
  : rclcpp::Node("simple_grasping_node", options)
{
  // Load parameters
  load_parameters(config_, this);

  // Initialize TF2 buffer and listener for cloud transformation
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize debug publisher if common.debug is true
  if (config_.common.debug)
  {
    debug_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/filtered_cloud", 10);
    debug_pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/markers", 10);  
}

  // Subscribe to input cloud topic
  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input_cloud", 10,
    std::bind(&SimpleGraspingNode::sensor_callback, this, _1)
  );

  // Create the action server for simple perception using a member callback for execution
  perception_action_server_ = rclcpp_action::create_server<SimplePerception>(
    this,
    "~/simple_perception",
    [this](const auto &, const auto &) -> rclcpp_action::GoalResponse {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [this](const auto &) -> rclcpp_action::CancelResponse {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    std::bind(&SimpleGraspingNode::execute_simple_perception, this, _1)
  );
}

void SimpleGraspingNode::sensor_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  latest_cloud_ = msg;
}

void SimpleGraspingNode::execute_simple_perception(const std::shared_ptr<GoalHandleSimplePerception> goal_handle)
{
  // Retrieve goal to use in sorting criteria
  auto goal = goal_handle->get_goal();
  auto result = std::make_shared<SimplePerception::Result>();

  if (!latest_cloud_) {
    result->success = false;
    result->message = "No point cloud available";
    goal_handle->abort(result);
    return;
  }

  // Transform the latest cloud to the target frame (config_.common.frame_id)
  sensor_msgs::msg::PointCloud2 transformed_cloud;
  geometry_msgs::msg::TransformStamped transformStamped;
  try {
    transformStamped = tf_buffer_->lookupTransform(
      config_.common.frame_id,
      latest_cloud_->header.frame_id,
      rclcpp::Time(0),
      rclcpp::Duration(1, 0)  // (seconds, nanoseconds)
    );
    tf2::doTransform(*latest_cloud_, transformed_cloud, transformStamped);
  } catch (tf2::TransformException &ex) {
    result->success = false;
    result->message = std::string("TF2 transform error: ") + ex.what();
    goal_handle->abort(result);
    return;
  }

  // Convert transformed ROS point cloud to PCL format
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(transformed_cloud, *pcl_cloud);

  // Apply filtering to the PCL cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = filters::filter_cloud(pcl_cloud, config_.filter);

  // Publish the filtered cloud if debug is enabled
  if (config_.common.debug && debug_pub_) {
    sensor_msgs::msg::PointCloud2 filtered_msg;
    pcl::toROSMsg(*filtered_cloud, filtered_msg);
    filtered_msg.header.frame_id = config_.common.frame_id;
    filtered_msg.header.stamp = latest_cloud_->header.stamp;
    debug_pub_->publish(filtered_msg);
  }

  // Detect planes using the plane detector
  std::vector<Plane> planes = plane_detector::detect_planes(filtered_cloud, config_.plane_detection, config_.common.n_threads);

  // Sort planes based on goal criteria:
  // If sort_planes_by_height is true, sort by plane center's z (lowest first),
  // else sort by Euclidean distance from goal->querry_point.
  if (goal->sort_planes_by_height) {
    std::sort(planes.begin(), planes.end(),
      [](const Plane &a, const Plane &b) {
        return a.obb.center.z < b.obb.center.z;
      });
  } else {
    std::sort(planes.begin(), planes.end(),
      [goal](const Plane &a, const Plane &b) {
        float da = std::sqrt(std::pow(a.obb.center.x - goal->querry_point.x, 2) +
                             std::pow(a.obb.center.y - goal->querry_point.y, 2) +
                             std::pow(a.obb.center.z - goal->querry_point.z, 2));
        float db = std::sqrt(std::pow(b.obb.center.x - goal->querry_point.x, 2) +
                             std::pow(b.obb.center.y - goal->querry_point.y, 2) +
                             std::pow(b.obb.center.z - goal->querry_point.z, 2));
        return da < db;
      });
  }

  // Convert each detected plane to the action result message format
  for (size_t i = 0; i < planes.size(); ++i) {
    simple_grasping_interfaces::msg::Plane plane_msg;
    
    // Convert plane inlier cloud to ROS message
    pcl::toROSMsg(*planes[i].inliers, plane_msg.cloud);
    plane_msg.cloud.header.frame_id = config_.common.frame_id;
    plane_msg.cloud.header.stamp = latest_cloud_->header.stamp;

    // Create marker for the plane OBB (blue color)
    auto marker = createMarkerFromOBB(planes[i].obb, "planes", static_cast<int>(i), 0.0f, 0.0f, 1.0f, 0.8f);
    plane_msg.obb = marker;

    // Copy plane equation coefficients
    plane_msg.a = planes[i].a;
    plane_msg.b = planes[i].b;
    plane_msg.c = planes[i].c;
    plane_msg.d = planes[i].d;

    result->planes.push_back(plane_msg);
  }

  // Publish debug markers 
  if (config_.common.debug && debug_pub_markers_) {
    visualization_msgs::msg::MarkerArray marker_array;
    for (const auto &plane_msg : result->planes) {
      marker_array.markers.push_back(plane_msg.obb);
    }
    debug_pub_markers_->publish(marker_array);
  }

  result->success = true;
  result->message = "Point cloud processed: transformed, filtered, and plane detection completed";
  goal_handle->succeed(result);
}

visualization_msgs::msg::Marker SimpleGraspingNode::createMarkerFromOBB(
    const OBB &obb,
    const std::string &ns,
    int id,
    float r, float g, float b, float a)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = config_.common.frame_id;
    marker.header.stamp = this->now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
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
    
    //marker.lifetime = rclcpp::Duration(10, 0);  // 10secs
    
    return marker;
  }