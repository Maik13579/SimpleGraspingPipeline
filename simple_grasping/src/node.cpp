#include "node.hpp"
#include "params.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std::placeholders;

SimpleGraspingNode::SimpleGraspingNode(const rclcpp::NodeOptions &options)
  : rclcpp::Node("simple_grasping_node", options)
{
  // Load parameters
  load_parameters(config_, this);

  // Initialize TF2 buffer and listener for cloud transformation
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize debug publisher if common.debug is true
  if (config_.common.debug)
  {
    debug_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/filtered_cloud", 10);
  }

  // Subscribe to input cloud topic
  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input_cloud", 10,
    std::bind(&SimpleGraspingNode::sensor_callback, this, _1)
  );

  // Create the action server for simple perception using a member callback for execution
  perception_action_server_ = rclcpp_action::create_server<SimplePerception>(
    this,
    "~/simple_perception",
    [this](const auto &, const auto &) -> rclcpp_action::GoalResponse {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [this](const auto &) -> rclcpp_action::CancelResponse {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    std::bind(&SimpleGraspingNode::execute_simple_perception, this, _1)
  );
}

void SimpleGraspingNode::sensor_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  latest_cloud_ = msg;
}

void SimpleGraspingNode::execute_simple_perception(const std::shared_ptr<GoalHandleSimplePerception> goal_handle)
{
  auto result = std::make_shared<SimplePerception::Result>();

  if (!latest_cloud_) {
    result->success = false;
    result->message = "No point cloud available";
    goal_handle->abort(result);
    return;
  }

  // Transform the latest cloud to the target frame (config_.common.frame_id)
  sensor_msgs::msg::PointCloud2 transformed_cloud;
  geometry_msgs::msg::TransformStamped transformStamped;
  try {
    transformStamped = tf_buffer_->lookupTransform(
      config_.common.frame_id,
      latest_cloud_->header.frame_id,
      rclcpp::Time(0),
      rclcpp::Duration(1, 0)  // Using (seconds, nanoseconds) constructor
    );
    tf2::doTransform(*latest_cloud_, transformed_cloud, transformStamped);
  } catch (tf2::TransformException &ex) {
    result->success = false;
    result->message = std::string("TF2 transform error: ") + ex.what();
    goal_handle->abort(result);
    return;
  }

  // Convert transformed ROS point cloud to PCL format
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(transformed_cloud, *pcl_cloud);

  // Apply filtering to the PCL cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = filters::filter_cloud(pcl_cloud, config_.filter);

  // Publish the filtered cloud if debug is enabled
  if (config_.common.debug && debug_pub_) {
    sensor_msgs::msg::PointCloud2 filtered_msg;
    pcl::toROSMsg(*filtered_cloud, filtered_msg);
    filtered_msg.header.frame_id = config_.common.frame_id;
    filtered_msg.header.stamp = latest_cloud_->header.stamp;
    debug_pub_->publish(filtered_msg);
  }

  result->success = true;
  result->message = "Point cloud transformed, filtered, and converted to PCL format";
  goal_handle->succeed(result);
}
