#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

#include <cmath>
#include <algorithm>

class OccupancyGrid : public rclcpp::Node
{
public:
    OccupancyGrid();
private:
    void update_cell(int index, float delta);
    void log_odds_to_grid();

    void pcl2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void raycast(int x0, int y0, int x1, int y1);
    void mark_free_circle(int cx, int cy, int radius);


    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl2_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    nav_msgs::msg::OccupancyGrid grid_;

    std::vector<float> log_odds;

    double resolution;
    double max_height;
    double min_height;
    double unknown_threshold;
    std::string map_frame;
};