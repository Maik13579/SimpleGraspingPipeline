#ifndef SIMPLE_GRASPING_WORLD_PARAMS_HPP
#define SIMPLE_GRASPING_WORLD_PARAMS_HPP

#include <rclcpp/rclcpp.hpp>
#include <type_traits>

template <typename T>
T declare_and_get_parameter(const std::string &param_name, const T &default_value, rclcpp::Node *node, const std::string &log_message)
{
    node->declare_parameter<T>(param_name, default_value);
    T param_value = node->get_parameter(param_name).get_value<T>();

    if constexpr (std::is_same<T, std::string>::value)
    {
        RCLCPP_INFO(node->get_logger(), "%s: %s", log_message.c_str(), param_value.c_str());
    }
    else
    {
        RCLCPP_INFO(node->get_logger(), "%s: %s", log_message.c_str(), std::to_string(param_value).c_str());
    }

    return param_value;
}

struct CommonParams
{
    std::string world_frame_id;
    std::string world_path;
    std::string tf_prefix;
};

struct FurnitureParams
{
    float voxel_resolution;
    float leaf_size;
    int grid_size;
};

struct NodeConfig
{
    CommonParams common;
    FurnitureParams furnitures;
};

void load_parameters(NodeConfig &config, rclcpp::Node *node);

#endif  // SIMPLE_GRASPING_WORLD_PARAMS_HPP