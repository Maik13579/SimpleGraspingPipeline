#ifndef SIMPLE_GRASPING_WORLD_UTILS_HPP
#define SIMPLE_GRASPING_WORLD_UTILS_HPP

#include <vector>
#include <stdexcept>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace utils {

    /**
     * @brief Split a point cloud into separate point clouds, each
     * containing points with a specific label.
     *
     * @param cloud The point cloud to split. It is expected to have a "label" field.
     * @param max_label The maximum label value to consider.
     * @param min_label The minimum label value to consider.
     * @return A vector of point clouds, where the i-th element contains points with the label (i+min_label).
     */
    std::vector<sensor_msgs::msg::PointCloud2> extractLabeledClouds(const sensor_msgs::msg::PointCloud2 &cloud, uint16_t max_label, uint16_t min_label = 0) {
        std::vector<sensor_msgs::msg::PointCloud2> labeled_clouds(max_label - min_label + 1);
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
            if (label < min_label || label > max_label) continue;
      
            auto &msg = labeled_clouds[label - min_label];
            msg.data.insert(msg.data.end(), point_ptr, point_ptr + cloud.point_step);
            msg.width++;
            msg.row_step = msg.point_step * msg.width;
        }
        return labeled_clouds;
    }

    /**
     * @brief Extract points from a point cloud with labels in a given range.
     *
     * @param cloud The point cloud to extract from. It is expected to have a "label" field.
     * @param max_label The maximum label value to consider.
     * @param min_label The minimum label value to consider.
     * @return A point cloud containing the points with labels in the range [min_label, max_label].
     */
    sensor_msgs::msg::PointCloud2 extractLabeledCloud(const sensor_msgs::msg::PointCloud2 &cloud, uint16_t max_label, uint16_t min_label = 0) {
        sensor_msgs::msg::PointCloud2 result;
        result.header = cloud.header;
        result.height = 1;
        result.is_dense = false;
        result.fields = cloud.fields;
        result.point_step = cloud.point_step;
        result.is_bigendian = cloud.is_bigendian;
      
        const uint32_t label_offset = [&]() {
            for (const auto &field : cloud.fields) {
                if (field.name == "label") return field.offset;
            }
            throw std::runtime_error("No label field in cloud");
        }();
      
        for (size_t i = 0; i < cloud.width * cloud.height; ++i) {
            const uint8_t *point_ptr = &cloud.data[i * cloud.point_step];
            uint16_t label = *reinterpret_cast<const uint16_t *>(point_ptr + label_offset);
            if (label < min_label || label > max_label) continue;
            result.data.insert(result.data.end(), point_ptr, point_ptr + cloud.point_step);
            result.width++;
            result.row_step = result.point_step * result.width;
        }
        return result;
    }
}

#endif // SIMPLE_GRASPING_WORLD_UTILS_HPP
