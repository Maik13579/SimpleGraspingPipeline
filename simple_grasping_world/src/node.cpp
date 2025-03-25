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

  std::filesystem::path objects_file = std::filesystem::path(config_.common.world_path) / "objects.yaml";
  objects_ = load_world_model(objects_file.string());

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
    std::bind(&SimpleGraspingWorldNode::load_objects, this)
  );
}
void SimpleGraspingWorldNode::load_objects()
{
  if (loaded_)
    return; //Shouldn't happen, but just in case

  if (!load_node_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "Waiting for /simple_world/component_manager/_container/load_node service...");
    return;
  }

  // Load objects
  for (auto &obj : objects_)
    load_object(obj);
  RCLCPP_INFO(this->get_logger(), "Loaded all objects.");

  // Cancel timer after first call (one-shot)
  timer_->cancel();
  loaded_ = true; // mark as loaded
}


void SimpleGraspingWorldNode::load_object(Object &obj)
{
  RCLCPP_INFO(this->get_logger(), "Loading object '%s'...", obj.id.c_str());
  load_object_component(obj);

  RCLCPP_INFO(this->get_logger(), "Creating service clients for object '%s'...", obj.id.c_str());

  std::string base_ns = "/simple_world/objects/" + obj.id + "/";
  obj.clients.add = this->create_client<pointcloud_server_interfaces::srv::Add>(base_ns + "add");
  obj.clients.clear = this->create_client<pointcloud_server_interfaces::srv::Clear>(base_ns + "clear");
  obj.clients.clear_points = this->create_client<pointcloud_server_interfaces::srv::ClearPoints>(base_ns + "clear_points");
  obj.clients.empty_around_point = this->create_client<pointcloud_server_interfaces::srv::EmptyAroundPoint>(base_ns + "empty_around_point");
  obj.clients.get = this->create_client<pointcloud_server_interfaces::srv::Get>(base_ns + "get");
  obj.clients.save = this->create_client<pointcloud_server_interfaces::srv::Save>(base_ns + "save");
  obj.clients.set_grid_size = this->create_client<pointcloud_server_interfaces::srv::SetGridSize>(base_ns + "set_grid_size");

  std::vector<std::pair<std::string, rclcpp::ClientBase::SharedPtr>> clients = {
    {"add", obj.clients.add},
    {"clear", obj.clients.clear},
    {"clear_points", obj.clients.clear_points},
    {"empty_around_point", obj.clients.empty_around_point},
    {"get", obj.clients.get},
    {"save", obj.clients.save},
    {"set_grid_size", obj.clients.set_grid_size},
  };

  for (const auto &[name, client] : clients)
  {
    RCLCPP_INFO(this->get_logger(), "Waiting for service '%s' of object '%s'...", name.c_str(), obj.id.c_str());
    client->wait_for_service();
    RCLCPP_INFO(this->get_logger(), "Connected to service '%s' for object '%s'", name.c_str(), obj.id.c_str());
  }
}



void SimpleGraspingWorldNode::load_object_component(Object &obj)
{
  auto req = std::make_shared<composition_interfaces::srv::LoadNode::Request>();
  req->package_name = "pointcloud_server";
  req->plugin_name = "pointcloud_server::PointcloudServerNode";
  req->node_name = obj.id;
  req->node_namespace = "/simple_world/objects";

  std::filesystem::path object_file = std::filesystem::path(config_.common.world_path) / (obj.id + ".pcd");

  std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("frame_id", config_.common.tf_prefix + obj.id),
    rclcpp::Parameter("map_path", object_file.string()),
    rclcpp::Parameter("only_services", true),
    rclcpp::Parameter("GridSize", config_.objects.grid_size),
    rclcpp::Parameter("VoxelResolution", config_.objects.voxel_resolution),
    rclcpp::Parameter("LeafSize", config_.objects.leaf_size),
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
    [&obj, this](rclcpp::Client<composition_interfaces::srv::LoadNode>::SharedFuture result) {
      if (result.get()->success) {
        obj.unique_component_id = result.get()->unique_id;
        RCLCPP_INFO(this->get_logger(), "Loaded node for object '%s' (component id: %lu)",
                    obj.id.c_str(), obj.unique_component_id);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to load node for object '%s'", obj.id.c_str());
      }
    }
  );
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(SimpleGraspingWorldNode)
