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

  // Initialize TF2 buffer and listener for cloud transformation
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Init service clients
  load_node_client_ = this->create_client<composition_interfaces::srv::LoadNode>("/simple_world/component_manager/_container/load_node");
  unload_node_client_ = this->create_client<composition_interfaces::srv::UnloadNode>("/simple_world/component_manager/_container/unload_node");
  start_perception_client_ = this->create_client<simple_grasping_interfaces::srv::StartPerception>("/simple_grasping_perception/start_perception");

  // Schedule deferred loading after initialization
  loaded_ = false;
  callback_group_timer_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000),
    std::bind(&SimpleGraspingWorldNode::load_furnitures, this),
    callback_group_timer_
  );
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
  std::vector<sensor_msgs::msg::PointCloud2> labeled_clouds(furniture.num_planes);
  for (auto &msg : labeled_clouds) {
    msg.header = cloud.header;
    msg.height = 1;
    msg.is_dense = false;
    msg.fields = cloud.fields;
    msg.point_step = cloud.point_step;
    msg.is_bigendian = cloud.is_bigendian;
  }

  const uint32_t label_offset = [&]() {
    for (const auto &field : cloud.fields) {
      if (field.name == "label") return field.offset;
    }
    throw std::runtime_error("No label field in cloud");
  }();

  for (size_t i = 0; i < cloud.width * cloud.height; ++i) {
    const uint8_t *point_ptr = &cloud.data[i * cloud.point_step];
    uint16_t label = *reinterpret_cast<const uint16_t *>(point_ptr + label_offset);
    if (label == 0 || label > furniture.num_planes) continue;

    auto &msg = labeled_clouds[label - 1];
    msg.data.insert(msg.data.end(), point_ptr, point_ptr + cloud.point_step);
    msg.width++;
    msg.row_step = msg.point_step * msg.width;
  }

  // Send each per-plane cloud to perception service
  for (size_t i = 0; i < labeled_clouds.size(); ++i) {
    auto req = std::make_shared<simple_grasping_interfaces::srv::StartPerception::Request>();
    req->cloud = labeled_clouds[i];
    req->only_planes = true;
    req->sort_planes_by_height = true;

    auto result = start_perception_client_->async_send_request(req);
    auto response = result.get();
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

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(SimpleGraspingWorldNode)
