#include "node.hpp"
#include "params.hpp"
#include "filters.hpp"
#include "plane_detector.hpp"
#include "object_detector.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <pcl/common/transforms.h>
#include <boost/shared_ptr.hpp>

using namespace std::placeholders;

SimpleGraspingNode::SimpleGraspingNode(const rclcpp::NodeOptions &options)
  : rclcpp::Node("simple_grasping_node", options)
{
  // Load parameters
  load_parameters(config_, this);

  // Load GPD
  grasp_detector_ = std::make_unique<gpd::GraspDetector>(config_.common.gpd_cfg_file);

  // disable plotting (breaks gpd currentlly)
  grasp_detector_->plot_normals_ = false;
  grasp_detector_->plot_samples_= false;
  grasp_detector_->plot_candidates_= false;
  grasp_detector_->plot_filtered_candidates_= false;
  grasp_detector_->plot_valid_grasps_= false;
  grasp_detector_->plot_clustered_grasps_= false;
  grasp_detector_->plot_selected_grasps_= false;

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

  // Create the action server
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

  grasp_action_server_ = rclcpp_action::create_server<SimpleGrasp>(
    this,
    "~/simple_grasp",
    [this](const auto &, const auto &) -> rclcpp_action::GoalResponse {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [this](const auto &) -> rclcpp_action::CancelResponse {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    std::bind(&SimpleGraspingNode::execute_simple_grasp, this, _1)
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

  /////////////////////////////////////////////////////////
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

  /////////////////////////////////////////////////////////
  // Detect planes
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
    //pcl::toROSMsg(*planes[i].inliers, plane_msg.cloud);
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

  // Publish debug markers for planes
  if (config_.common.debug && debug_pub_markers_) {
    visualization_msgs::msg::MarkerArray marker_array;
    for (const auto &plane_msg : result->planes) {
      marker_array.markers.push_back(plane_msg.obb);
    }
    debug_pub_markers_->publish(marker_array);
  }

  /////////////////////////////////////////////////////////
  // Object detection:
  // Use additional goal parameters: height_above_plane and width_adjustment
  float height_above_plane = (goal->height_above_plane > 0.0f) ? goal->height_above_plane : 0.3f;
  std::vector<Object> objects = object_detector::detect_objects(
    filtered_cloud, planes, config_.object_detection,
    height_above_plane, goal->width_adjustment, config_.common.n_threads
  );

  // Sort objects by Euclidean distance from goal->querry_point
  std::sort(objects.begin(), objects.end(),
    [goal](const Object &a, const Object &b) {
      float da = std::sqrt(std::pow(a.obb.center.x - goal->querry_point.x, 2) +
                           std::pow(a.obb.center.y - goal->querry_point.y, 2) +
                           std::pow(a.obb.center.z - goal->querry_point.z, 2));
      float db = std::sqrt(std::pow(b.obb.center.x - goal->querry_point.x, 2) +
                           std::pow(b.obb.center.y - goal->querry_point.y, 2) +
                           std::pow(b.obb.center.z - goal->querry_point.z, 2));
      return da < db;
    });

  // Fill object results in the action message
  for (size_t i = 0; i < objects.size(); ++i) {
    simple_grasping_interfaces::msg::Object obj_msg;
    
    // Convert object inlier cloud to ROS message
    //pcl::toROSMsg(*objects[i].cloud, obj_msg.cloud);
    obj_msg.cloud.header.frame_id = config_.common.frame_id;
    obj_msg.cloud.header.stamp = latest_cloud_->header.stamp;
    
    // Create marker for the object OBB (red color)
    obj_msg.obb = createMarkerFromOBB(objects[i].obb, "objects", static_cast<int>(i), 1.0f, 0.0f, 0.0f, 0.8f);
    
    // Store the index of the plane on which the object is detected
    obj_msg.plane_index = objects[i].plane_index;
    
    result->objects.push_back(obj_msg);
  }

  // Publish AOI markers (in cyan)
  if (config_.common.debug && debug_pub_markers_) {
    visualization_msgs::msg::MarkerArray aoi_marker_array;
    for (size_t i = 0; i < planes.size(); ++i) {
      auto marker = createMarkerFromOBB(planes[i].aoi, "aois", static_cast<int>(i),
                                        0.0f, 1.0f, 1.0f, 0.2f); // Hell cyan with low alpha
      aoi_marker_array.markers.push_back(marker);
    }
    debug_pub_markers_->publish(aoi_marker_array);
  }

  // Publish debug markers for objects
  if (config_.common.debug && debug_pub_markers_) {
    visualization_msgs::msg::MarkerArray obj_marker_array;
    for (const auto &obj_msg : result->objects) {
      obj_marker_array.markers.push_back(obj_msg.obb);
    }
    debug_pub_markers_->publish(obj_marker_array);
  }

  // Store planes and objects
  planes_ = planes;
  objects_ = objects;

  result->success = true;
  result->message = "Point cloud processed: transformed, filtered, and object detection completed";
  goal_handle->succeed(result);
}


void SimpleGraspingNode::execute_simple_grasp(const std::shared_ptr<GoalHandleSimpleGrasp> goal_handle)
{
  auto goal = goal_handle->get_goal();
  auto result = std::make_shared<SimpleGrasp::Result>();

  if (goal->object_index < 0 || goal->object_index >= static_cast<int>(objects_.size())) {
    result->success = false;
    result->message = "Invalid object index";
    goal_handle->abort(result);
    return;
  }

  const Object &selected_obj = objects_[goal->object_index];
  pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  int idx_counter = 0;
  std::vector<int> selected_obj_indices;

  // Add object clouds
  for (size_t i = 0; i < objects_.size(); ++i) {
    if (objects_[i].plane_index == selected_obj.plane_index && objects_[i].cloud) {
      if (static_cast<int>(i) == goal->object_index) {
        for (size_t pt = 0; pt < objects_[i].cloud->points.size(); ++pt)
          selected_obj_indices.push_back(idx_counter + pt);
      }
      idx_counter += objects_[i].cloud->points.size();
      *combined_cloud += *(objects_[i].cloud);
    }
  }

  // Add grid-based plane cloud from the supporting plane
  if (selected_obj.plane_index < static_cast<int>(planes_.size())) {
    const Plane &plane = planes_[selected_obj.plane_index];
    double grid_res = 0.025; // Grid resolution in meters
    double dx = plane.obb.max_pt.x - plane.obb.min_pt.x;
    double dy = plane.obb.max_pt.y - plane.obb.min_pt.y;

    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (double x = -dx; x <= dx; x += grid_res) {
      for (double y = -dy; y <= dy; y += grid_res) {
        Eigen::Vector3f local_point(static_cast<float>(x), static_cast<float>(y), 0.0f);
        Eigen::Vector3f global_point = plane.obb.center.getVector3fMap() + plane.obb.rotation * local_point;
        pcl::PointXYZ pt;
        pt.x = global_point.x();
        pt.y = global_point.y();
        pt.z = global_point.z();
        plane_cloud->points.push_back(pt);
      }
    }
    *combined_cloud += *plane_cloud;

    // Create a copy of the plane and move it up by min_distance_to_plane
    if (goal->min_distance_to_plane > 0.0){
      pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud_above(new pcl::PointCloud<pcl::PointXYZ>());
      for (const auto &pt : plane_cloud->points) {
        pcl::PointXYZ pt_above = pt;
        pt_above.z += goal->min_distance_to_plane;
        plane_cloud_above->points.push_back(pt_above);
      }
      *combined_cloud += *plane_cloud_above;
    }

    // If disable_top_grasp is true, add an upward copy of the plane cloud.
    if (goal->disable_top_grasp) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud_above(new pcl::PointCloud<pcl::PointXYZ>());
      for (const auto &pt : plane_cloud->points) {
        pcl::PointXYZ pt_above = pt;
        pt_above.z += (selected_obj.obb.max_pt.z - selected_obj.obb.min_pt.z);
        plane_cloud_above->points.push_back(pt_above);
      }
      *combined_cloud += *plane_cloud_above;
    }
  }

  // Publish combined cloud
  if (config_.common.debug && debug_pub_) {
    sensor_msgs::msg::PointCloud2 debug_msg;
    pcl::toROSMsg(*combined_cloud, debug_msg);
    debug_msg.header.frame_id = config_.common.frame_id;
    debug_msg.header.stamp = this->now();
    debug_pub_->publish(debug_msg);
  }

  // Transform the combined cloud into the object frame
  Eigen::Matrix3f R_inv = selected_obj.obb.rotation.transpose();
  Eigen::Vector3f center(selected_obj.obb.center.x, selected_obj.obb.center.y, selected_obj.obb.center.z);
  pcl::transformPointCloud(*combined_cloud, *combined_cloud,
                           Eigen::Affine3f(Eigen::Translation3f(-R_inv * center) * R_inv));

  combined_cloud_rgb_ = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
  pcl::copyPointCloud(*combined_cloud, *combined_cloud_rgb_);

  geometry_msgs::msg::TransformStamped sensor_to_target = tf_buffer_->lookupTransform(
      config_.common.frame_id, latest_cloud_->header.frame_id, rclcpp::Time(0), rclcpp::Duration(1, 0));

  view_points_.resize(3, 1);
  view_points_ << sensor_to_target.transform.translation.x,
                  sensor_to_target.transform.translation.y,
                  sensor_to_target.transform.translation.z;

  camera_source_ = Eigen::MatrixXi::Ones(1, combined_cloud_rgb_->size());
  gpd_cloud_ = std::make_shared<gpd::util::Cloud>(combined_cloud_rgb_, camera_source_, view_points_);
  gpd_cloud_->setSampleIndices(selected_obj_indices);
 
  // Update approach direction 
  // Use the provided approach_direction if non-zero, otherwise default to (1,0,0).
  Eigen::Vector3f action_approach(goal->approach_direction.x,
    goal->approach_direction.y,
    goal->approach_direction.z);
  if (action_approach.norm() < 1e-6) {
    action_approach = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
  }
  // Transform the action approach direction into the object frame.
  Eigen::Vector3f new_direction = R_inv * action_approach;
  grasp_detector_->direction_ = new_direction.cast<double>();

  // Update approach direction filtering:
  // If thresh_rad is 0.0, disable filtering; otherwise, enable it and set the threshold.
  if (std::abs(goal->thresh_rad) < 1e-6) {
    grasp_detector_->filter_approach_direction_ = false;
  } else {
    grasp_detector_->filter_approach_direction_ = true;
    grasp_detector_->thresh_rad_ = goal->thresh_rad;
  }

  grasp_detector_->num_selected_ = goal->num_grasps_selected;

  grasp_detector_->preprocessPointCloud(*gpd_cloud_);
  if (gpd_cloud_->getCloudProcessed()->empty() || gpd_cloud_->getSampleIndices().empty()) {
    result->success = false;
    result->message = "Preprocessed cloud or indices empty!";
    goal_handle->abort(result);
    return;
  }

  auto grasps = grasp_detector_->detectGrasps(*gpd_cloud_);
  RCLCPP_INFO(get_logger(), "Detected %zu grasps from GPD.", grasps.size());

  // Publish grasps
  if (config_.common.debug && debug_pub_markers_) {
    Eigen::Matrix3f R_obj = selected_obj.obb.rotation;
    Eigen::Vector3f center(selected_obj.obb.center.x, selected_obj.obb.center.y, selected_obj.obb.center.z);
    Eigen::Affine3f T_obj_inv(Eigen::Translation3f(center) * R_obj);
    visualization_msgs::msg::MarkerArray all_grasp_markers;
    int marker_id = 0;
    for (const auto &grasp : grasps) {
      auto markers = createGraspMarker(*grasp, marker_id, config_.common.frame_id, T_obj_inv);
      for (const auto &m : markers.markers)
      {
        marker_id++; //increment marker id
        all_grasp_markers.markers.push_back(m);
      }
    }
    debug_pub_markers_->publish(all_grasp_markers);
  }

  result->success = true;
  result->message = "Grasps detected";
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

  visualization_msgs::msg::MarkerArray SimpleGraspingNode::createGraspMarker(
    const gpd::candidate::Hand &grasp,
    int id,
    const std::string &frame_id,
    const Eigen::Affine3f &T_obj_inv)
  {
    
    // Get Hand geometry
    auto hand_geometry = grasp_detector_->getHandSearchParameters().hand_geometry_;
    double hand_depth = hand_geometry.depth_;
    double hand_height = hand_geometry.height_;
    double outer_diameter = hand_geometry.outer_diameter_;
    double finger_width = hand_geometry.finger_width_;
    double hw = 0.5 * outer_diameter - 0.5 * finger_width;  // half-width
  
    // Retrieve grasp candidate properties
    Eigen::Vector3d pos_d = grasp.getPosition();
    Eigen::Vector3d binormal_d = grasp.getBinormal();
    Eigen::Vector3d approach_d = grasp.getApproach();
    Eigen::Matrix3d hand_frame_d = grasp.getFrame();
  
    // Cast to float (to match our transform T_obj_inv)
    Eigen::Vector3f pos = pos_d.cast<float>();
    Eigen::Vector3f binormal = binormal_d.cast<float>();
    Eigen::Vector3f approach = approach_d.cast<float>();
    Eigen::Matrix3f hand_frame = hand_frame_d.cast<float>();
  
    // Compute key points in object frame
    Eigen::Vector3f left_bottom_obj  = pos - hw * binormal;
    Eigen::Vector3f right_bottom_obj = pos + hw * binormal;
    Eigen::Vector3f left_top_obj     = left_bottom_obj + hand_depth * approach;
    Eigen::Vector3f right_top_obj    = right_bottom_obj + hand_depth * approach;
    Eigen::Vector3f left_center_obj  = left_bottom_obj + 0.5f * (left_top_obj - left_bottom_obj);
    Eigen::Vector3f right_center_obj = right_bottom_obj + 0.5f * (right_top_obj - right_bottom_obj);
    Eigen::Vector3f base_center_obj  = left_bottom_obj + 0.5f * (right_bottom_obj - left_bottom_obj) - 0.01f * approach;
    Eigen::Vector3f approach_center_obj = base_center_obj - 0.04f * approach;
  
    // Define dimensions for markers (length, width, height)
    Eigen::Vector3f finger_lwh; finger_lwh << hand_depth, finger_width, hand_height;
    Eigen::Vector3f approach_lwh; approach_lwh << 0.08, finger_width, hand_height;
  
    // Transform key points from object frame to global frame using T_obj_inv
    Eigen::Vector3f left_bottom  = T_obj_inv * left_bottom_obj;
    Eigen::Vector3f right_bottom = T_obj_inv * right_bottom_obj;
    Eigen::Vector3f left_center  = T_obj_inv * left_center_obj;
    Eigen::Vector3f right_center = T_obj_inv * right_center_obj;
    Eigen::Vector3f base_center  = T_obj_inv * base_center_obj;
    Eigen::Vector3f approach_center = T_obj_inv * approach_center_obj;
    // Transform the hand frame: global hand frame = rotation part of T_obj_inv * hand_frame.
    Eigen::Matrix3f global_hand_frame = T_obj_inv.linear() * hand_frame;
  
    visualization_msgs::msg::MarkerArray marker_array;
  
    // Helper lambda: creates a cube marker for a finger or approach indicator.
    auto createFingerMarker = [&](const Eigen::Vector3f &center,
                                  const Eigen::Matrix3f &frame,
                                  const Eigen::Vector3f &lwh,
                                  int marker_id) -> visualization_msgs::msg::Marker {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame_id;
      marker.header.stamp = now();
      marker.ns = "grasp";
      marker.id = marker_id;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = center.x();
      marker.pose.position.y = center.y();
      marker.pose.position.z = center.z();
      Eigen::Quaternionf quat(frame);
      marker.pose.orientation.x = quat.x();
      marker.pose.orientation.y = quat.y();
      marker.pose.orientation.z = quat.z();
      marker.pose.orientation.w = quat.w();
      marker.scale.x = lwh.x();
      marker.scale.y = lwh.y();
      marker.scale.z = lwh.z();
      // Color will be set later based on confidence.
      marker.color.a = 0.5;
      return marker;
    };
  
    // Helper lambda: creates a base marker as a cube.
    auto createBaseMarker = [&](const Eigen::Vector3f &start,
                                const Eigen::Vector3f &end,
                                const Eigen::Matrix3f &frame,
                                float fixed_length,
                                float height,
                                int marker_id) -> visualization_msgs::msg::Marker {
      Eigen::Vector3f center = start + 0.5f * (end - start);
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame_id;
      marker.header.stamp = now();
      marker.ns = "grasp";
      marker.id = marker_id;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = center.x();
      marker.pose.position.y = center.y();
      marker.pose.position.z = center.z();
      Eigen::Quaternionf quat(frame);
      marker.pose.orientation.x = quat.x();
      marker.pose.orientation.y = quat.y();
      marker.pose.orientation.z = quat.z();
      marker.pose.orientation.w = quat.w();
      marker.scale.x = fixed_length;
      marker.scale.y = (end - start).norm();
      marker.scale.z = height;
      marker.color.a = 0.5;
      return marker;
    };
  
    // Create markers in global frame.
    visualization_msgs::msg::Marker left_finger = createFingerMarker(left_center, global_hand_frame, finger_lwh, id);
    visualization_msgs::msg::Marker right_finger = createFingerMarker(right_center, global_hand_frame, finger_lwh, id + 1);
    visualization_msgs::msg::Marker approach_marker = createFingerMarker(approach_center, global_hand_frame, approach_lwh, id + 2);
    visualization_msgs::msg::Marker base_marker = createBaseMarker(left_bottom, right_bottom, global_hand_frame, 0.02, hand_height, id + 3);
  
    // Set color based on grasp confidence (assume grasp.getScore() returns a value in [0,1])
    double conf = grasp.getScore();
    auto setColor = [conf](visualization_msgs::msg::Marker &m) {
      m.color.r = 1.0 - conf;  // lower confidence -> more red
      m.color.g = conf;        // higher confidence -> more green
      m.color.b = 0.0;
      m.color.a = 0.5;
    };
    setColor(left_finger);
    setColor(right_finger);
    setColor(approach_marker);
    setColor(base_marker);
  
    marker_array.markers.push_back(left_finger);
    marker_array.markers.push_back(right_finger);
    marker_array.markers.push_back(approach_marker);
    marker_array.markers.push_back(base_marker);
  
    return marker_array;
  }
