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

  // Schedule deferred loading after initialization
  loaded_ = false;
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000),
    std::bind(&SimpleGraspingWorldNode::load_furnitures, this)
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
