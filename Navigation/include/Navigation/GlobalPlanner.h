#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <vector>
#include <queue>
#include <mutex>
#include <string>
#include <utility>
#include <limits>
#include <cmath>
#include <algorithm>
#include <yaml-cpp/yaml.h>
#include <chrono>


class GlobalPlanner : public rclcpp::Node
{
public:
    GlobalPlanner();

private:
    struct GridNode
    {
        int x;
        int y;
        float g;
        float h;
    };

    struct OpenEntry
    {
        float f;
        int idx;
        bool operator>(const OpenEntry & other) const { return f > other.f; }
    };

    struct Waypoint
    {
        double x;
        double y;
    };

    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void planning_loop();

    bool load_waypoints_from_yaml(const std::string & path, std::vector<Waypoint> & out) const;
    bool plan_waypoints(const nav_msgs::msg::OccupancyGrid & grid, const std::vector<Waypoint> & waypoints);

    bool get_robot_pose(double & x, double & y);
    bool world_to_grid(const nav_msgs::msg::OccupancyGrid & grid, double wx, double wy, int & gx, int & gy) const;
    void grid_to_world(const nav_msgs::msg::OccupancyGrid & grid, int gx, int gy, double & wx, double & wy) const;
    bool in_bounds(const nav_msgs::msg::OccupancyGrid & grid, int gx, int gy) const;
    int flat_index(const nav_msgs::msg::OccupancyGrid & grid, int gx, int gy) const;
    bool is_occupied(const nav_msgs::msg::OccupancyGrid & grid, int gx, int gy) const;

    bool plan_astar(
        const nav_msgs::msg::OccupancyGrid & grid,
        int sx,
        int sy,
        int gx,
        int gy,
        std::vector<std::pair<int, int>> & out_cells
    ) const;

    void publish_path(const nav_msgs::msg::OccupancyGrid & grid, const std::vector<std::pair<int, int>> & cells);

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
    mutable std::mutex map_mutex_;

    std::string map_topic_;
    std::string goal_topic_;
    std::string path_topic_;
    std::string map_frame_;
    std::string base_frame_;

    std::string waypoints_file_;
    bool use_waypoints_;
    std::vector<Waypoint> waypoints_;

    rclcpp::TimerBase::SharedPtr timer_;
    bool path_active_ = false;
    bool map_received_ = false;

    int occupied_threshold_;
    bool unknown_is_occupied_;
};