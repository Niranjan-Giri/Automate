#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <vector>
#include <cmath>

class LocalCostmap : public rclcpp::Node
{
public:
    LocalCostmap();

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void inflate_obstacles();
    void publish_costmap(rclcpp::Time stamp);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl2_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;

    std::vector<int8_t> cost_data;
    int width, height;
    float resolution;
    float origin_x, origin_y;
};