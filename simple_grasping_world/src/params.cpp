#include "params.hpp"

void load_parameters(NodeConfig &config, rclcpp::Node *node)
{
    // Common parameters
    config.common.world_frame_id = declare_and_get_parameter<std::string>("common.world_frame_id", "map", node, "common.world_frame_id");
    config.common.world_path = declare_and_get_parameter<std::string>("common.world_path", "", node, "common.world_path");
    config.common.tf_prefix = declare_and_get_parameter<std::string>("common.tf_prefix", "world_", node, "common.tf_prefix");

    // furnitureects parameters
    config.furnitureects.voxel_resolution = declare_and_get_parameter<float>("furnitureects.voxel_resolution", 0.1f, node, "furnitureects.voxel_resolution");
    config.furnitureects.leaf_size = declare_and_get_parameter<float>("furnitureects.leaf_size", 0.01f, node, "furnitureects.leaf_size");
    config.furnitureects.grid_size = declare_and_get_parameter<int>("furnitureects.grid_size", 50, node, "furnitureects.grid_size");
}
