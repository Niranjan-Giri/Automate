#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <mutex>

class MapTF: public rclcpp::Node
{
public:
    MapTF();

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void scan_match_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr scan_match_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::string odom_topic;
    std::string scan_match_topic;

    tf2::Transform current_map_pose_;
    std::mutex pose_mutex_;
};