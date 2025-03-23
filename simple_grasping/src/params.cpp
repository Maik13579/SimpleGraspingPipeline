#include "params.hpp"

void load_parameters(NodeConfig &config, rclcpp::Node *node)
{
    // Common parameters
    config.common.frame_id = declare_and_get_parameter<std::string>("common.frame_id", "base_footprint", node, "common.frame_id");
    config.common.n_threads = declare_and_get_parameter<int>("common.n_threads", 1, node, "common.n_threads");
    config.common.debug = declare_and_get_parameter<bool>("common.debug", false, node, "common.debug");
    config.common.gpd_cfg_file = declare_and_get_parameter<std::string>("common.gpd_cfg_file", "/gpd/cfg/ros_vino_params.cfg", node, "common.gpd_cfg_file");

    // Filter parameters
    config.filter.voxel_size = declare_and_get_parameter<float>("filter.voxel_size", 0.005f, node, "filter.voxel_size");
    config.filter.radius = declare_and_get_parameter<float>("filter.radius", 0.05f, node, "filter.radius");
    config.filter.min_points = declare_and_get_parameter<int>("filter.min_points", 3, node, "filter.min_points");
    config.filter.workspace.x_min = declare_and_get_parameter<float>("filter.workspace.x_min", -1.0f, node, "filter.workspace.x_min");
    config.filter.workspace.x_max = declare_and_get_parameter<float>("filter.workspace.x_max", 1.0f, node, "filter.workspace.x_max");
    config.filter.workspace.y_min = declare_and_get_parameter<float>("filter.workspace.y_min", -1.0f, node, "filter.workspace.y_min");
    config.filter.workspace.y_max = declare_and_get_parameter<float>("filter.workspace.y_max", 1.0f, node, "filter.workspace.y_max");
    config.filter.workspace.z_min = declare_and_get_parameter<float>("filter.workspace.z_min", 0.1f, node, "filter.workspace.z_min");
    config.filter.workspace.z_max = declare_and_get_parameter<float>("filter.workspace.z_max", 2.0f, node, "filter.workspace.z_max");

    // Plane detection parameters
    config.plane_detection.normal_estimation_radius = declare_and_get_parameter<float>("plane_detection.normal_estimation_radius", 0.1f, node, "plane_detection.normal_estimation_radius");
    config.plane_detection.horizontal_threshold = declare_and_get_parameter<float>("plane_detection.horizontal_threshold", 2.0f, node, "plane_detection.horizontal_threshold");
    config.plane_detection.clustering_tolerance = declare_and_get_parameter<float>("plane_detection.clustering_tolerance", 0.05f, node, "plane_detection.clustering_tolerance");
    config.plane_detection.clustering_min_cluster_size = declare_and_get_parameter<int>("plane_detection.clustering_min_cluster_size", 20, node, "plane_detection.clustering_min_cluster_size");
    config.plane_detection.clustering_max_cluster_size = declare_and_get_parameter<int>("plane_detection.clustering_max_cluster_size", 1000000, node, "plane_detection.clustering_max_cluster_size");
    config.plane_detection.min_area = declare_and_get_parameter<float>("plane_detection.min_area", 0.1f, node, "plane_detection.min_area");
    config.plane_detection.plane_thickness = declare_and_get_parameter<float>("plane_detection.plane_thickness", 0.02f, node, "plane_detection.plane_thickness");
    config.plane_detection.ransac_iterations = declare_and_get_parameter<int>("plane_detection.ransac_iterations", 100, node, "plane_detection.ransac_iterations");

    // Object detection parameters
    config.object_detection.clustering_tolerance = declare_and_get_parameter<float>("object_detection.clustering_tolerance", 0.025f, node, "object_detection.object_clustering_tolerance");
    config.object_detection.clustering_min_cluster_size = declare_and_get_parameter<int>("object_detection.clustering_min_cluster_size", 10, node, "object_detection.object_clustering_min_cluster_size");
    config.object_detection.clustering_max_cluster_size = declare_and_get_parameter<int>("object_detection.clustering_max_cluster_size", 5000, node, "object_detection.object_clustering_max_cluster_size");
}
