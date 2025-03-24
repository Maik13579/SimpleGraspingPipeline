#include "node.hpp"
#include "utils.hpp"
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

  // Publisher for best grasp
  grasp_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/grasp", 10);
  pre_grasp_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/pre_grasp", 10);
  retreat_grasp_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/retreat_grasp", 10);

  // Hand Publisher and hand marker
  hand_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/hand", 10);  
  const auto &hand_geometry = grasp_detector_->getHandSearchParameters().hand_geometry_;
  hand_ = utils::createHandMarker(0, hand_geometry);
  for (auto &marker : hand_.markers) {
    marker.header.frame_id = config_.common.grasp_frame_id;
  }

  // Create service servers instead of action servers.
  perception_srv_ = this->create_service<simple_grasping_interfaces::srv::StartPerception>(
    "~/start_perception",
    std::bind(&SimpleGraspingNode::handleStartPerception, this, std::placeholders::_1, std::placeholders::_2)
  );

  grasp_srv_ = this->create_service<simple_grasping_interfaces::srv::GenerateGrasps>(
    "~/generate_grasps",
    std::bind(&SimpleGraspingNode::handleGenerateGrasps, this, std::placeholders::_1, std::placeholders::_2)
  );
}

void SimpleGraspingNode::sensor_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  latest_cloud_ = msg;
  hand_pub_->publish(hand_);
}

void SimpleGraspingNode::handleStartPerception(
  const std::shared_ptr<simple_grasping_interfaces::srv::StartPerception::Request> request,
  std::shared_ptr<simple_grasping_interfaces::srv::StartPerception::Response> response)
{
  auto start_time = std::chrono::steady_clock::now();

  if (!latest_cloud_) {
    response->success = false;
    response->message = "No point cloud available";
    return;
  }

  // Transform the latest cloud to the target frame.
  sensor_msgs::msg::PointCloud2 transformed_cloud;
  geometry_msgs::msg::TransformStamped transformStamped;
  try {
    transformStamped = tf_buffer_->lookupTransform(
      config_.common.frame_id,
      latest_cloud_->header.frame_id,
      rclcpp::Time(0),
      rclcpp::Duration(1, 0)
    );
    tf2::doTransform(*latest_cloud_, transformed_cloud, transformStamped);
  } catch (tf2::TransformException &ex) {
    response->success = false;
    response->message = std::string("TF2 transform error: ") + ex.what();
    return;
  }

  // Convert ROS cloud to PCL format.
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(transformed_cloud, *pcl_cloud);

  // Filter the cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = filters::filter_cloud(pcl_cloud, config_.filter);

  if (config_.common.debug && debug_pub_) {
    sensor_msgs::msg::PointCloud2 filtered_msg;
    pcl::toROSMsg(*filtered_cloud, filtered_msg);
    filtered_msg.header.frame_id = config_.common.frame_id;
    filtered_msg.header.stamp = latest_cloud_->header.stamp;
    debug_pub_->publish(filtered_msg);
  }

  // Detect planes.
  std::vector<Plane> planes = plane_detector::detect_planes(filtered_cloud, config_.plane_detection, config_.common.n_threads);

  // Sort planes.
  if (request->sort_planes_by_height) { //By height
    std::sort(planes.begin(), planes.end(),
      [](const Plane &a, const Plane &b) {
        return a.obb.center.z < b.obb.center.z;
      });
  } else { // By Distance to querry point
    std::sort(planes.begin(), planes.end(),
      [request](const Plane &a, const Plane &b) {
        float da = std::sqrt(std::pow(a.obb.center.x - request->querry_point.x, 2) +
                             std::pow(a.obb.center.y - request->querry_point.y, 2) +
                             std::pow(a.obb.center.z - request->querry_point.z, 2));
        float db = std::sqrt(std::pow(b.obb.center.x - request->querry_point.x, 2) +
                             std::pow(b.obb.center.y - request->querry_point.y, 2) +
                             std::pow(b.obb.center.z - request->querry_point.z, 2));
        return da < db;
      });
  }

  // Convert planes to result message format.
  for (size_t i = 0; i < planes.size(); ++i) {
    simple_grasping_interfaces::msg::Plane plane_msg;
    if (request->return_cloud && planes[i].inliers)
      pcl::toROSMsg(*(planes[i].inliers), plane_msg.cloud);
    plane_msg.cloud.header.frame_id = config_.common.frame_id;
    plane_msg.cloud.header.stamp = latest_cloud_->header.stamp;
    plane_msg.obb = utils::createMarkerFromOBB(planes[i].obb, "planes", static_cast<int>(i), 0.0f, 0.0f, 1.0f, 0.8f);
    plane_msg.obb.header.stamp = latest_cloud_->header.stamp;
    plane_msg.obb.header.frame_id = config_.common.frame_id;
    plane_msg.a = planes[i].a;
    plane_msg.b = planes[i].b;
    plane_msg.c = planes[i].c;
    plane_msg.d = planes[i].d;
    response->planes.push_back(plane_msg);
  }

  // Publish debug markers for planes.
  if (config_.common.debug && debug_pub_markers_) {
    visualization_msgs::msg::MarkerArray marker_array;
    for (const auto &plane_msg : response->planes) {
      marker_array.markers.push_back(plane_msg.obb);
    }
    debug_pub_markers_->publish(marker_array);
  }

  // Object detection using additional parameters.
  float height_above_plane = (request->height_above_plane > 0.0f) ? request->height_above_plane : 0.3f;
  std::vector<Object> objects = object_detector::detect_objects(
    filtered_cloud, planes, config_.object_detection,
    height_above_plane, request->width_adjustment, config_.common.n_threads
  );

  // Sort objects by distance to querry point
  std::sort(objects.begin(), objects.end(),
    [request](const Object &a, const Object &b) {
      float da = std::sqrt(std::pow(a.obb.center.x - request->querry_point.x, 2) +
                           std::pow(a.obb.center.y - request->querry_point.y, 2) +
                           std::pow(a.obb.center.z - request->querry_point.z, 2));
      float db = std::sqrt(std::pow(b.obb.center.x - request->querry_point.x, 2) +
                           std::pow(b.obb.center.y - request->querry_point.y, 2) +
                           std::pow(b.obb.center.z - request->querry_point.z, 2));
      return da < db;
    });

  // Convert objects to response message format.
  for (size_t i = 0; i < objects.size(); ++i) {
    simple_grasping_interfaces::msg::Object obj_msg;
    if (request->return_cloud && objects[i].cloud)
      pcl::toROSMsg(*(objects[i].cloud), obj_msg.cloud);
    obj_msg.cloud.header.frame_id = config_.common.frame_id;
    obj_msg.cloud.header.stamp = latest_cloud_->header.stamp;
    obj_msg.obb = utils::createMarkerFromOBB(objects[i].obb, "objects", static_cast<int>(i), 1.0f, 0.0f, 0.0f, 0.8f);
    obj_msg.obb.header.stamp = latest_cloud_->header.stamp;
    obj_msg.obb.header.frame_id = config_.common.frame_id;
    obj_msg.plane_index = objects[i].plane_index;
    response->objects.push_back(obj_msg);
  }

  // Publish AOI and object markers if debug enabled.
  if (config_.common.debug && debug_pub_markers_) {
    visualization_msgs::msg::MarkerArray aoi_marker_array;
    for (size_t i = 0; i < planes.size(); ++i) {
      auto marker = utils::createMarkerFromOBB(planes[i].aoi, "aois", static_cast<int>(i),
                                        0.0f, 1.0f, 1.0f, 0.2f);
      marker.header.stamp = latest_cloud_->header.stamp;
      marker.header.frame_id = config_.common.frame_id;
      aoi_marker_array.markers.push_back(marker);
    }
    debug_pub_markers_->publish(aoi_marker_array);

    visualization_msgs::msg::MarkerArray obj_marker_array;
    for (const auto &obj_msg : response->objects) {
      obj_marker_array.markers.push_back(obj_msg.obb);
    }
    debug_pub_markers_->publish(obj_marker_array);
  }

  // Store planes and objects.
  planes_ = planes;
  objects_ = objects;

  response->success = true;
  response->message = "";

  auto end_time = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
  RCLCPP_INFO(this->get_logger(), "Perception took %ld ms", elapsed);
}

void SimpleGraspingNode::handleGenerateGrasps(
  const std::shared_ptr<simple_grasping_interfaces::srv::GenerateGrasps::Request> request,
  std::shared_ptr<simple_grasping_interfaces::srv::GenerateGrasps::Response> response)
{
  auto start_time = std::chrono::steady_clock::now();

  if (request->object_index < 0 || request->object_index >= static_cast<int>(objects_.size())) {
    response->success = false;
    response->message = "Invalid object index";
    return;
  }

  // Shortcut to selected object.
  const Object &selected_obj = objects_[request->object_index];


  /////////////////////////////////////////////////////////////
  //
  //  GENERATE CLOUD
  //
  /////////////////////////////////////////////////////////////
  pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  int idx_counter = 0;
  std::vector<int> selected_obj_indices;
  // Generate combined cloud from all objects on the same plane as selected.
  for (size_t i = 0; i < objects_.size(); ++i) {
    if (objects_[i].plane_index == selected_obj.plane_index) {
      // Choose cloud pointer: sample from OBB if requested, otherwise use the object's cloud.
      auto cur_cloud = request->sample_cloud_from_obb ? 
          utils::createPointCloudFromOBB(objects_[i].obb, 0.01f) : objects_[i].cloud;
      if (cur_cloud) {
        if (static_cast<int>(i) == request->object_index) {
          for (size_t pt = 0; pt < cur_cloud->points.size(); ++pt)
            selected_obj_indices.push_back(idx_counter + pt);
        }
        idx_counter += cur_cloud->points.size();
        *combined_cloud += *cur_cloud;
      }
    }
  }
  
  // Add synthetic plane cloud from the supporting plane.
  if (selected_obj.plane_index < static_cast<int>(planes_.size())) {
    const Plane &plane = planes_[selected_obj.plane_index];
    double grid_res = 0.01; // Grid resolution in meters
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

    if (request->min_distance_to_plane > 0.0) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud_above(new pcl::PointCloud<pcl::PointXYZ>());
      for (const auto &pt : plane_cloud->points) {
        pcl::PointXYZ pt_above = pt;
        pt_above.z += request->min_distance_to_plane;
        plane_cloud_above->points.push_back(pt_above);
      }
      *combined_cloud += *plane_cloud_above;
    }

    if (request->disable_top_grasp) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud_top(new pcl::PointCloud<pcl::PointXYZ>());
      for (const auto &pt : plane_cloud->points) {
        pcl::PointXYZ pt_top = pt;
        pt_top.z += (selected_obj.obb.max_pt.z - selected_obj.obb.min_pt.z);
        plane_cloud_top->points.push_back(pt_top);
      }
      *combined_cloud += *plane_cloud_top;
    }
  }

  if (config_.common.debug && debug_pub_) {
    sensor_msgs::msg::PointCloud2 debug_msg;
    pcl::toROSMsg(*combined_cloud, debug_msg);
    debug_msg.header.frame_id = config_.common.frame_id;
    debug_msg.header.stamp = this->now();
    debug_pub_->publish(debug_msg);
  }

  /////////////////////////////////////////////////////////////
  //
  //  TRANSFORMATION TO OBJECT COORDINATES
  //
  /////////////////////////////////////////////////////////////
  // Compute transformation T that maps points from global (config_.common.frame_id) into object coordinates.
  Eigen::Matrix3f R_inv = selected_obj.obb.rotation.transpose();
  Eigen::Vector3f obj_center(selected_obj.obb.center.x,
                              selected_obj.obb.center.y,
                              selected_obj.obb.center.z);
  Eigen::Affine3f T = Eigen::Affine3f(Eigen::Translation3f(-R_inv * obj_center) * R_inv);
  pcl::transformPointCloud(*combined_cloud, *combined_cloud, T);

  // Create a deep copy for GPD.
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr combined_cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGBA>());
  pcl::copyPointCloud(*combined_cloud, *combined_cloud_rgb);

  // Lookup sensor origin in global frame and transform it.
  geometry_msgs::msg::TransformStamped sensor_to_target = tf_buffer_->lookupTransform(
      config_.common.frame_id, latest_cloud_->header.frame_id, rclcpp::Time(0), rclcpp::Duration(1, 0));
  Eigen::Vector3f sensor_origin(
      sensor_to_target.transform.translation.x,
      sensor_to_target.transform.translation.y,
      sensor_to_target.transform.translation.z);
  Eigen::Vector3f transformed_sensor_origin = T * sensor_origin;
  Eigen::MatrixXd view_points(3, 1);
  view_points << transformed_sensor_origin.x(),
                       transformed_sensor_origin.y(),
                       transformed_sensor_origin.z();



  /////////////////////////////////////////////////////////////
  //
  //  GPD SETUP
  //
  /////////////////////////////////////////////////////////////
  // Create a new Camera Source matrix.
  Eigen::MatrixXi camera_source = Eigen::MatrixXi::Ones(1, combined_cloud_rgb->size());

  // Initialize gpd cloud.
  gpd_cloud_ = NULL;
  gpd_cloud_ = new gpd::util::Cloud(combined_cloud_rgb, camera_source, view_points);
  gpd_cloud_->setSampleIndices(selected_obj_indices);

  // Update approach direction.
  Eigen::Vector3f action_approach(request->approach_direction.x,
                                  request->approach_direction.y,
                                  request->approach_direction.z);
  if (action_approach.norm() < 1e-6) {
    action_approach = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
  }
  Eigen::Vector3f new_direction = R_inv * action_approach;
  grasp_detector_->direction_ = new_direction.cast<double>();

  if (std::abs(request->thresh_deg) < 1e-6) {
    grasp_detector_->filter_approach_direction_ = false;
  } else {
    grasp_detector_->filter_approach_direction_ = true;
    grasp_detector_->thresh_rad_ = request->thresh_deg * 0.0174532925f; // Pi/180
  }

  grasp_detector_->num_selected_ = request->num_grasps_selected;

  /////////////////////////////////////////////////////////////
  //
  //  GPD
  //
  /////////////////////////////////////////////////////////////
  grasp_detector_->preprocessPointCloud(*gpd_cloud_);
  if (gpd_cloud_->getCloudProcessed()->empty() || gpd_cloud_->getSampleIndices().empty()) {
    response->success = false;
    response->message = "Preprocessed cloud or indices empty!";
    return;
  }

  // Detect grasps with GPD.
  std::vector<std::unique_ptr<gpd::candidate::Hand>> grasps = grasp_detector_->detectGrasps(*gpd_cloud_);
  RCLCPP_INFO(this->get_logger(), "Detected %zu grasps from GPD.", grasps.size());

  // Publish grasp markers in debug mode.
  if (config_.common.debug && debug_pub_markers_) {
    // Normalize scores.
    double min_score = std::numeric_limits<double>::max();
    double max_score = std::numeric_limits<double>::lowest();
    for (const auto &g : grasps) {
      double s = g->getScore();
      min_score = std::min(min_score, s);
      max_score = std::max(max_score, s);
    }

    // Object transformation (to map grasp from object frame to global)
    Eigen::Matrix3f R_obj = selected_obj.obb.rotation;
    Eigen::Vector3f center(selected_obj.obb.center.x,
                          selected_obj.obb.center.y,
                          selected_obj.obb.center.z);
    Eigen::Affine3f T_obj_inv(Eigen::Translation3f(center) * R_obj);

    visualization_msgs::msg::MarkerArray all_grasp_markers;
    int marker_id = 0;

    for (const auto &g : grasps) {
      double normalized_score = (g->getScore() - min_score) / (max_score - min_score + 1e-6);
      
      // Compute grasp frame as an affine transform
      Eigen::Matrix3f R_grasp = g->getFrame().cast<float>();
      Eigen::Vector3f t_grasp = g->getPosition().cast<float>();
      Eigen::Affine3f T_grasp = Eigen::Affine3f(Eigen::Translation3f(t_grasp) * R_grasp);

      // Compose transforms: local → grasp → global
      Eigen::Affine3f T_global = T_obj_inv * T_grasp;

      // Transform hand_markers to global frame
      auto global_markers = utils::transformMarkerArray(hand_, T_global);

      // Set frame ID and color
      for (auto &marker : global_markers.markers) {
        marker.header.frame_id = config_.common.frame_id;
        marker.header.stamp = now();
        marker.color.r = static_cast<float>(1.0 - normalized_score);
        marker.color.g = static_cast<float>(normalized_score);
        marker.color.b = 0.0f;
        marker.color.a = static_cast<float>(normalized_score);
        marker.id = marker_id++;

        all_grasp_markers.markers.push_back(marker);
      }
    }
    debug_pub_markers_->publish(all_grasp_markers);
  }


  /////////////////////////////////////////////////////////////
  //
  //  COMPUTE PRE_GRASP, GRASP, AND RETREAT POSES
  //
  /////////////////////////////////////////////////////////////
  // Compute pre_grasp, grasp, and retreat poses for each candidate.
  Plane supporting_plane = planes_[selected_obj.plane_index];
  Eigen::Vector3f plane_normal(supporting_plane.a, supporting_plane.b, supporting_plane.c);
  plane_normal.normalize();

  std::vector<simple_grasping_interfaces::msg::Grasp> grasp_msgs;
  bool published = false;
  for (const auto &g : grasps) {
    // Candidate position in object coordinates.
    Eigen::Vector3f grasp_obj = g->getPosition().cast<float>();
    Eigen::Vector3f grasp_global = T.inverse() * grasp_obj;

    // Candidate rotation.
    Eigen::Matrix3f candidate_rot_obj = g->getFrame().cast<float>();
    Eigen::Matrix3f candidate_rot_global = T.inverse().linear() * candidate_rot_obj;
    Eigen::Quaternionf quat_global(candidate_rot_global);

    geometry_msgs::msg::PoseStamped grasp_pose;
    grasp_pose.header.frame_id = config_.common.frame_id;
    grasp_pose.header.stamp = this->now();
    grasp_pose.pose.position.x = grasp_global.x();
    grasp_pose.pose.position.y = grasp_global.y();
    grasp_pose.pose.position.z = grasp_global.z();
    grasp_pose.pose.orientation.x = quat_global.x();
    grasp_pose.pose.orientation.y = quat_global.y();
    grasp_pose.pose.orientation.z = quat_global.z();
    grasp_pose.pose.orientation.w = quat_global.w();

    // Pre-grasp pose.
    Eigen::Vector3f approach_obj = g->getApproach().cast<float>().normalized();
    Eigen::Vector3f pre_grasp_obj = grasp_obj - request->pre_grasp_dist * approach_obj;
    Eigen::Vector3f pre_grasp_global = T.inverse() * pre_grasp_obj;
    geometry_msgs::msg::PoseStamped pre_grasp_pose;
    pre_grasp_pose.header = grasp_pose.header;
    pre_grasp_pose.pose.position.x = pre_grasp_global.x();
    pre_grasp_pose.pose.position.y = pre_grasp_global.y();
    pre_grasp_pose.pose.position.z = pre_grasp_global.z();
    pre_grasp_pose.pose.orientation = grasp_pose.pose.orientation;

    // Retreat pose.
    Eigen::Vector3f retreat_global = grasp_global + request->retreat_dist * plane_normal;
    geometry_msgs::msg::PoseStamped retreat_pose;
    retreat_pose.header = grasp_pose.header;
    retreat_pose.pose.position.x = retreat_global.x();
    retreat_pose.pose.position.y = retreat_global.y();
    retreat_pose.pose.position.z = retreat_global.z();
    retreat_pose.pose.orientation = grasp_pose.pose.orientation;

    simple_grasping_interfaces::msg::Grasp grasp_msg;
    grasp_msg.pre_grasp = pre_grasp_pose;
    grasp_msg.grasp = grasp_pose;
    grasp_msg.retreat = retreat_pose;
    grasp_msg.score = g->getScore();
    grasp_msg.is_full_antipodal = g->isFullAntipodal();
    grasp_msg.is_half_antipodal = g->isHalfAntipodal();
    grasp_msg.grasp_width = g->getGraspWidth();

    grasp_msgs.push_back(grasp_msg);

    if (!published) {
      grasp_pub_->publish(grasp_pose);
      pre_grasp_pub_->publish(pre_grasp_pose);
      retreat_grasp_pub_->publish(retreat_pose);
      published = true;
    }
  }

  response->grasps = grasp_msgs;
  response->success = true;
  response->message = "";
  auto end_time = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
  RCLCPP_INFO(this->get_logger(), "Grasp generation took %ld ms", elapsed);
}
