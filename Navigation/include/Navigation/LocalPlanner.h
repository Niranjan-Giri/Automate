#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <mutex>
#include <string>
#include <chrono>
#include <cmath>


class LocalPlanner : public rclcpp::Node
{
public:
	LocalPlanner();

private:
	void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
	void control_loop();

	bool get_robot_pose(double & x, double & y, double & yaw) const;
	bool find_lookahead_point(
		const nav_msgs::msg::Path & path,
		double robot_x,
		double robot_y,
		double robot_yaw,
		geometry_msgs::msg::PoseStamped & target,
		double & target_distance,
		double & target_lateral
	) const;

	rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
	rclcpp::TimerBase::SharedPtr timer_;

	std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

	nav_msgs::msg::Path::SharedPtr latest_path_;
	mutable std::mutex path_mutex_;

	std::string path_topic_;
	std::string cmd_topic_;
	std::string map_frame_;
	std::string base_frame_;

	double lookahead_distance_;
	double goal_tolerance_;
	double linear_speed_;
	double control_rate_hz_;
};
