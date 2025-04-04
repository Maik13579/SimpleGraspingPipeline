#include "params.hpp"

void load_parameters(NodeConfig &config, rclcpp::Node *node)
{
    // Common parameters
    config.common.world_frame_id = declare_and_get_parameter<std::string>("common.world_frame_id", "map", node, "common.world_frame_id");
    config.common.world_path = declare_and_get_parameter<std::string>("common.world_path", "", node, "common.world_path");
    config.common.tf_prefix = declare_and_get_parameter<std::string>("common.tf_prefix", "world_", node, "common.tf_prefix");

    // furnitures parameters
    config.furnitures.voxel_resolution = declare_and_get_parameter<float>("furnitures.voxel_resolution", 0.1f, node, "furnitures.voxel_resolution");
    config.furnitures.leaf_size = declare_and_get_parameter<float>("furnitures.leaf_size", 0.01f, node, "furnitures.leaf_size");
    config.furnitures.grid_size = declare_and_get_parameter<int>("furnitures.grid_size", 50, node, "furnitures.grid_size");
}
