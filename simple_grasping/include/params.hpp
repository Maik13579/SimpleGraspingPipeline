#ifndef SIMPLE_GRASPING_PARAMS_HPP
#define SIMPLE_GRASPING_PARAMS_HPP

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
    std::string frame_id;
    std::string grasp_frame_id;
    int n_threads;
    bool debug;
    std::string gpd_cfg_file;
};

struct Workspace
{
    float x_min;
    float x_max;
    float y_min;
    float y_max;
    float z_min;
    float z_max;
};

struct FilterParams
{
    float voxel_size;
    float radius;
    int min_points;
    Workspace workspace;
};

struct PlaneParams
{
    float normal_estimation_radius;
    float horizontal_threshold;
    float clustering_tolerance;
    float clustering_min_cluster_size;
    float clustering_max_cluster_size;
    float min_area;
    float plane_thickness;
    float ransac_iterations;
};

struct ObjectParams
{
    float clustering_tolerance;
    float clustering_min_cluster_size;
    float clustering_max_cluster_size;
};

struct NodeConfig
{
    CommonParams common;
    FilterParams filter;
    PlaneParams plane_detection;
    ObjectParams object_detection;
};

void load_parameters(NodeConfig &config, rclcpp::Node *node);

#endif  // SIMPLE_GRASPING_PARAMS_HPP