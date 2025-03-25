#include "params.hpp"

void load_parameters(NodeConfig &config, rclcpp::Node *node)
{
    // Common parameters
    config.common.world_frame_id = declare_and_get_parameter<std::string>("common.world_frame_id", "map", node, "common.world_frame_id");
    config.common.world_path = declare_and_get_parameter<std::string>("common.world_path", "", node, "common.world_path");
    config.common.tf_prefix = declare_and_get_parameter<std::string>("common.tf_prefix", "world_", node, "common.tf_prefix");

    // Objects parameters
    config.objects.voxel_resolution = declare_and_get_parameter<float>("objects.voxel_resolution", 0.1f, node, "objects.voxel_resolution");
    config.objects.leaf_size = declare_and_get_parameter<float>("objects.leaf_size", 0.01f, node, "objects.leaf_size");
    config.objects.grid_size = declare_and_get_parameter<int>("objects.grid_size", 50, node, "objects.grid_size");
}
