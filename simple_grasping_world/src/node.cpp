#include "node.hpp"
#include <filesystem>

#include "pointcloud_server_interfaces/srv/add.hpp"
#include "pointcloud_server_interfaces/srv/clear.hpp"
#include "pointcloud_server_interfaces/srv/clear_points.hpp"
#include "pointcloud_server_interfaces/srv/empty_around_point.hpp"
#include "pointcloud_server_interfaces/srv/get.hpp"
#include "pointcloud_server_interfaces/srv/save.hpp"
#include "pointcloud_server_interfaces/srv/set_grid_size.hpp"

using namespace std::placeholders;

SimpleGraspingWorldNode::SimpleGraspingWorldNode(const rclcpp::NodeOptions &options)
  : rclcpp::Node("simple_grasping_world", options)
{
  // Load parameters
  load_parameters(config_, this);

  std::filesystem::path furnitures_file = std::filesystem::path(config_.common.world_path) / "furnitures.yaml";
  furnitures_ = load_world_model(furnitures_file.string());

  // Init subscribers
  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("~/input_cloud", 10, std::bind(&SimpleGraspingWorldNode::pointcloud_callback, this, _1));

  // Initialize TF2 buffer and listener for cloud transformation
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Init service clients
  load_node_client_ = this->create_client<composition_interfaces::srv::LoadNode>("/simple_world/component_manager/_container/load_node");
  unload_node_client_ = this->create_client<composition_interfaces::srv::UnloadNode>("/simple_world/component_manager/_container/unload_node");
  start_perception_client_ = this->create_client<simple_grasping_interfaces::srv::StartPerception>("/simple_grasping_perception/start_perception");

  // Init service server
  callback_group_add_frame_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  add_frame_srv_ = this->create_service<simple_grasping_interfaces::srv::AddFrame>(
    "~/add_frame",
    std::bind(&SimpleGraspingWorldNode::add_frame_callback, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile(),
    callback_group_add_frame_);


  // Schedule deferred loading after initialization
  loaded_ = false;
  callback_group_timer_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000),
    std::bind(&SimpleGraspingWorldNode::load_furnitures, this),
    callback_group_timer_
  );
}

void SimpleGraspingWorldNode::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  latest_cloud_ = *msg;
}
void SimpleGraspingWorldNode::load_furnitures()
{
  if (loaded_)
    return; //Shouldn't happen, but just in case

  if (!load_node_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "Waiting for /simple_world/component_manager/_container/load_node service...");
    return;
  }

  if (!start_perception_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "Waiting for /simple_grasping_perception/start_perception service...");
    return;
  }

  // Load furnitures
  for (auto &furniture : furnitures_)
    load_furniture(furniture);
  RCLCPP_INFO(this->get_logger(), "Loaded all furnitures.");

  // Cancel timer after first call (one-shot)
  timer_->cancel();
  loaded_ = true; // mark as loaded
}


void SimpleGraspingWorldNode::load_furniture(Furniture &furniture)
{
  RCLCPP_INFO(this->get_logger(), "Loading furniture '%s'...", furniture.id.c_str());
  load_furniture_component(furniture);

  RCLCPP_INFO(this->get_logger(), "Creating service clients for furniture '%s'...", furniture.id.c_str());

  std::string base_ns = "/simple_world/furnitures/" + furniture.id + "/";
  furniture.clients.add = this->create_client<pointcloud_server_interfaces::srv::Add>(base_ns + "add");
  furniture.clients.clear = this->create_client<pointcloud_server_interfaces::srv::Clear>(base_ns + "clear");
  furniture.clients.clear_points = this->create_client<pointcloud_server_interfaces::srv::ClearPoints>(base_ns + "clear_points");
  furniture.clients.empty_around_point = this->create_client<pointcloud_server_interfaces::srv::EmptyAroundPoint>(base_ns + "empty_around_point");
  furniture.clients.get = this->create_client<pointcloud_server_interfaces::srv::Get>(base_ns + "get");
  furniture.clients.save = this->create_client<pointcloud_server_interfaces::srv::Save>(base_ns + "save");
  furniture.clients.set_grid_size = this->create_client<pointcloud_server_interfaces::srv::SetGridSize>(base_ns + "set_grid_size");

  std::vector<std::pair<std::string, rclcpp::ClientBase::SharedPtr>> clients = {
    {"add", furniture.clients.add},
    {"clear", furniture.clients.clear},
    {"clear_points", furniture.clients.clear_points},
    {"empty_around_point", furniture.clients.empty_around_point},
    {"get", furniture.clients.get},
    {"save", furniture.clients.save},
    {"set_grid_size", furniture.clients.set_grid_size},
  };

  for (const auto &[name, client] : clients)
  {
    RCLCPP_INFO(this->get_logger(), "Waiting for service '%s' of furniture '%s'...", name.c_str(), furniture.id.c_str());
    client->wait_for_service();
    RCLCPP_INFO(this->get_logger(), "Connected to service '%s' for furniture '%s'", name.c_str(), furniture.id.c_str());
  }

  // Use get service to get the points
  auto get_req = std::make_shared<pointcloud_server_interfaces::srv::Get::Request>();
  auto get_result = furniture.clients.get->async_send_request(get_req);
  auto result = get_result.get(); // This blocks so make sure to use a multi-threaded container
  
  if (!result->success) {
    RCLCPP_WARN(this->get_logger(), "Failed to get cloud for '%s': %s", furniture.id.c_str(), result->message.c_str());
    return;
  }
  
  const auto &cloud = result->cloud;
  if (cloud.data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Cloud for '%s' is empty, skipping perception", furniture.id.c_str());
    return;
  }

  // Split labeled cloud into per-plane clouds
  std::vector<sensor_msgs::msg::PointCloud2> labeled_clouds = utils::extractLabeledClouds(cloud, furniture.num_planes, 1);

  // Send each per-plane cloud to perception service
  for (size_t i = 0; i < labeled_clouds.size(); ++i) {
    auto response = start_perception(labeled_clouds[i], true);
    if (!response->success) {
      RCLCPP_WARN(this->get_logger(), "Perception failed for plane %ld of furniture '%s'", i + 1, furniture.id.c_str());
      continue;
    }

    // Store result in plane database
    for (const auto &plane : response->planes) {
      Plane p;
      p.obb = plane.obb;
      p.height = plane.obb.pose.position.z;
      p.furniture_id = furniture.id;
      plane_db_.insert(p);
    }
  }
}



void SimpleGraspingWorldNode::load_furniture_component(Furniture &furniture)
{
  auto req = std::make_shared<composition_interfaces::srv::LoadNode::Request>();
  req->package_name = "pointcloud_server";
  req->plugin_name = "pointcloud_server::PointcloudServerNode";
  req->node_name = furniture.id;
  req->node_namespace = "/simple_world/furnitures";

  std::filesystem::path furniture_file = std::filesystem::path(config_.common.world_path) / (furniture.id + ".pcd");

  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("frame_id", config_.common.tf_prefix + furniture.id),
    rclcpp::Parameter("map_path", furniture_file.string()),
    rclcpp::Parameter("only_services", true),
    rclcpp::Parameter("GridSize", config_.furnitures.grid_size),
    rclcpp::Parameter("VoxelResolution", config_.furnitures.voxel_resolution),
    rclcpp::Parameter("LeafSize", config_.furnitures.leaf_size),
    rclcpp::Parameter("MinProbabilityPerVoxel", 0.0),
    rclcpp::Parameter("DecayingThreshold", -1.0),
    rclcpp::Parameter("PublishFrequency", 0.2), // all 5 seconds
    rclcpp::Parameter("Sampling", 1), // SamplingMode::Keep Last
    rclcpp::Parameter("ExpandOption", false),
    rclcpp::Parameter("RollOption", false),
    rclcpp::Parameter("ProbabilityToIntensity", false)
  };

  for (const auto &p : params)
    req->parameters.push_back(p.to_parameter_msg());

  // Remappings
  req->remap_rules.push_back("~/map:=~/cloud");
  // hide unused stuff
  req->remap_rules.push_back("~/submap:=~/_submap");
  req->remap_rules.push_back("~/build_kd_tree:=~/_build_kd_tree");
  req->remap_rules.push_back("~/build_submap:=~/_build_submap");
  req->remap_rules.push_back("~/get_submap:=~/_get_submap");
  req->remap_rules.push_back("~/knn_search:=~/_knn_search");
  req->remap_rules.push_back("~/label_new_points:=~/_label_new_points");
  req->remap_rules.push_back("~/set_leaf_size:=~/_set_leaf_size");
  req->remap_rules.push_back("~/set_voxel_resolution:=~/_set_voxel_resolution");
  req->remap_rules.push_back("~/roll:=~/_roll");
  req->remap_rules.push_back("~/reset:=~/_reset");

  auto future = load_node_client_->async_send_request(req,
    [&furniture, this](rclcpp::Client<composition_interfaces::srv::LoadNode>::SharedFuture result) {
      if (result.get()->success) {
        furniture.unique_component_id = result.get()->unique_id;
        RCLCPP_INFO(this->get_logger(), "Loaded node for furniture '%s' (component id: %lu)",
                    furniture.id.c_str(), furniture.unique_component_id);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to load node for furniture '%s'", furniture.id.c_str());
      }
    }
  );
}


void SimpleGraspingWorldNode::add_frame_callback(
  const std::shared_ptr<simple_grasping_interfaces::srv::AddFrame::Request> request,
  std::shared_ptr<simple_grasping_interfaces::srv::AddFrame::Response> response)
{
  // Get the input cloud from the request (or fallback to stored cloud)
  sensor_msgs::msg::PointCloud2 input_cloud;
  if (request->cloud.header.frame_id.empty()) {
    // latest_cloud_ is stored as a value.
    if (latest_cloud_.data.empty()) {
      response->success = false;
      response->message = "No cloud provided and no stored cloud available";
      return;
    } else {
      input_cloud = latest_cloud_;
      RCLCPP_INFO(this->get_logger(), "Using stored cloud as input.");
    }
  } else {
    input_cloud = request->cloud;
    RCLCPP_INFO(this->get_logger(), "Using provided cloud as input.");
  }

  // Lookup transform from input cloud frame to world frame.
  geometry_msgs::msg::TransformStamped transformStamped;
  try {
    transformStamped = tf_buffer_->lookupTransform(
      config_.common.world_frame_id,
      input_cloud.header.frame_id,
      input_cloud.header.stamp,
      rclcpp::Duration(1, 0)
    );
    RCLCPP_INFO(this->get_logger(), "Obtained transform from '%s' to world frame.", input_cloud.header.frame_id.c_str());
  } catch (tf2::TransformException &ex) {
    response->success = false;
    response->message = std::string("TF transform error: ") + ex.what();
    return;
  }

  // Call the perception service
  auto perception_res = start_perception(input_cloud); 
  if (!perception_res->success) {
    response->success = false;
    response->message = perception_res->message;
    RCLCPP_ERROR(this->get_logger(), "Perception failed: %s", perception_res->message.c_str());
    return;
  }
  
  RCLCPP_INFO(this->get_logger(), "Perception returned %zu planes and %zu objects.", 
    perception_res->planes.size(), perception_res->objects.size());

  // For each plane
  std::vector<simple_grasping_interfaces::msg::Plane> transformed_planes;
  std::set<std::string> found_furniture_ids;
  for (const auto &plane : perception_res->planes){

    // Transform to world frame
    auto transformed_plane = transformPlane(plane, transformStamped);
    transformed_planes.push_back(transformed_plane);

    // Convert to internal Plane struct.
    Plane new_plane;
    new_plane.obb = transformed_plane.obb;
    new_plane.height = transformed_plane.obb.pose.position.z;
    new_plane.furniture_id = "";  // Initially unset.

    // Query the database for planes near this height.
    auto touching = plane_db_.query(new_plane, 0.01f); // Todo parameterize threshold.
    if (touching.empty()) {// No touching planes found: create a new plane.
      new_plane.furniture_id = "TOBEASSIGNED"; // mark as "to be assigned"

    } else if (touching.size() == 1) {// Exactly one touching plane found: use it.
      found_furniture_ids.insert(touching.front().furniture_id);

    } else {// Multiple touching planes: print TODO message.
      RCLCPP_WARN(rclcpp::get_logger("SimpleGraspingWorldNode"), //TODO
                  "TODO: merge planes - multiple touching planes detected for new plane");
      found_furniture_ids.insert(touching.front().furniture_id); // use the first for now
    }

    // Check if we need to assign a furniture id to the new plane.
    if (new_plane.furniture_id == "TOBEASSIGNED") {
      std::set<std::string> f_ids;

      // get all planes in the database and check if they touch
      auto all_planes = plane_db_.get_sorted_planes();
      for (const auto &db_plane : all_planes) {
        if (planes_touch(new_plane, db_plane, 0.01)) {
          f_ids.insert(db_plane.furniture_id);
        }
      }
      if (f_ids.empty()) { // No touching furniture found: create a new furniture
        RCLCPP_WARN(rclcpp::get_logger("SimpleGraspingWorldNode"), //TODO
                    "TODO: create new furniture");

      } else if (f_ids.size() == 1) { // Exactly one touching furniture found: use it
        new_plane.furniture_id = *f_ids.begin();
        found_furniture_ids.insert(new_plane.furniture_id);

      } else { // Multiple touching furnitures: merge all furnitures
        RCLCPP_WARN(rclcpp::get_logger("SimpleGraspingWorldNode"), //TODO
                    "TODO: merge furnitures - multiple furniture ids detected for new plane");
        new_plane.furniture_id = *f_ids.begin(); // use the first for now
        found_furniture_ids.insert(new_plane.furniture_id);
      }
      plane_db_.insert(new_plane); // add new plane to database
    }
  }

  // For each detected object, transform both the marker and the inlier cloud.
  std::vector<simple_grasping_interfaces::msg::Object> transformed_objects;
  for (const auto &object : perception_res->objects)
    transformed_objects.push_back(transformObject(object, transformStamped));
  


  response->success = true;
  response->message = "";
}

std::shared_ptr<simple_grasping_interfaces::srv::StartPerception::Response> 
SimpleGraspingWorldNode::start_perception(
  const sensor_msgs::msg::PointCloud2 &cloud,
  bool only_planes,
  bool sort_planes_by_height,
  double height_above_plane,
  double width_adjustment,
  bool return_cloud) 
{
  auto perception_req = std::make_shared<simple_grasping_interfaces::srv::StartPerception::Request>();
  perception_req->cloud = cloud;
  perception_req->only_planes = only_planes;
  perception_req->sort_planes_by_height = sort_planes_by_height;
  perception_req->height_above_plane = height_above_plane;
  perception_req->width_adjustment = width_adjustment;
  perception_req->return_cloud = return_cloud;
  RCLCPP_INFO(this->get_logger(), "Calling perception service...");
  
  auto future = start_perception_client_->async_send_request(perception_req);
  auto perception_res = future.get(); // blocks until the result is available  
  return perception_res;
}



#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(SimpleGraspingWorldNode)
