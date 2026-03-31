#include "Navigation/LocalPlanner.h"

LocalPlanner::LocalPlanner() : Node("local_planner")
{
	this->declare_parameter<std::string>("path_topic", "/global_path");
	this->declare_parameter<std::string>("cmd_topic", "/cmd_vel");
	this->declare_parameter<std::string>("map_frame", "map");
	this->declare_parameter<std::string>("base_frame", "base_link");
	this->declare_parameter<double>("lookahead_distance", 1.0);
	this->declare_parameter<double>("goal_tolerance", 0.3);
	this->declare_parameter<double>("linear_speed", 0.1);
	this->declare_parameter<double>("control_rate_hz", 20.0);

	path_topic_ = this->get_parameter("path_topic").as_string();
	cmd_topic_ = this->get_parameter("cmd_topic").as_string();
	map_frame_ = this->get_parameter("map_frame").as_string();
	base_frame_ = this->get_parameter("base_frame").as_string();
	lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
	goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
	linear_speed_ = this->get_parameter("linear_speed").as_double();
	control_rate_hz_ = this->get_parameter("control_rate_hz").as_double();
	if (control_rate_hz_ <= 0.0)
	{
		control_rate_hz_ = 20.0;
	}

	tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

	path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
		path_topic_,
		10,
		std::bind(&LocalPlanner::path_callback, this, std::placeholders::_1)
	);

	cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, 10);

	const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
	timer_ = this->create_wall_timer(
		std::chrono::duration_cast<std::chrono::milliseconds>(period),
		std::bind(&LocalPlanner::control_loop, this)
	);

	RCLCPP_INFO(
		this->get_logger(),
		"Local planner started. path=%s cmd=%s",
		path_topic_.c_str(),
		cmd_topic_.c_str()
	);
}

void LocalPlanner::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
	std::lock_guard<std::mutex> lock(path_mutex_);
	latest_path_ = msg;
}

void LocalPlanner::control_loop()
{
	nav_msgs::msg::Path::SharedPtr path_copy;
	{
		std::lock_guard<std::mutex> lock(path_mutex_);
		path_copy = latest_path_;
	}

	if (!path_copy || path_copy->poses.empty())
	{
		return;
	}

	double robot_x = 0.0;
	double robot_y = 0.0;
	double robot_yaw = 0.0;
	if (!get_robot_pose(robot_x, robot_y, robot_yaw))
	{
		RCLCPP_DEBUG(this->get_logger(), "Failed to get robot pose.");
		return;
	}

	const auto & goal_pose = path_copy->poses.back().pose.position;
	const double goal_dx = goal_pose.x - robot_x;
	const double goal_dy = goal_pose.y - robot_y;
	const double goal_dist = std::hypot(goal_dx, goal_dy);

	if (goal_dist < goal_tolerance_)
	{
		geometry_msgs::msg::Twist stop_cmd;
		cmd_pub_->publish(stop_cmd);
		return;
	}

	geometry_msgs::msg::PoseStamped target;
	double target_distance = 0.0;
	double target_lateral = 0.0;
	if (!find_lookahead_point(*path_copy, robot_x, robot_y, robot_yaw, target, target_distance, target_lateral))
	{
		return;
	}

	if (target_distance < 1e-3)
	{
		return;
	}

	const double curvature = (2.0 * target_lateral) / (target_distance * target_distance);

	geometry_msgs::msg::Twist cmd;
	cmd.linear.x = linear_speed_;
	cmd.angular.z = curvature * cmd.linear.x;
	cmd_pub_->publish(cmd);
}

bool LocalPlanner::get_robot_pose(double & x, double & y, double & yaw) const
{
	try
	{
		geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
			map_frame_,
			base_frame_,
			tf2::TimePointZero,
			tf2::durationFromSec(0.1)
		);

		x = tf.transform.translation.x;
		y = tf.transform.translation.y;
		yaw = tf2::getYaw(tf.transform.rotation);
		return true;
	}
	catch (const tf2::TransformException & e)
	{
		RCLCPP_DEBUG(this->get_logger(), "TF lookup failed: %s", e.what());
		return false;
	}
}

bool LocalPlanner::find_lookahead_point(
	const nav_msgs::msg::Path & path,
	double robot_x,
	double robot_y,
	double robot_yaw,
	geometry_msgs::msg::PoseStamped & target,
	double & target_distance,
	double & target_lateral
) const
{
	const double cos_yaw = std::cos(robot_yaw);
	const double sin_yaw = std::sin(robot_yaw);

	for (const auto & pose : path.poses)
	{
		const double dx = pose.pose.position.x - robot_x;
		const double dy = pose.pose.position.y - robot_y;
		const double distance = std::hypot(dx, dy);

		const double x_r = cos_yaw * dx + sin_yaw * dy;
		const double y_r = -sin_yaw * dx + cos_yaw * dy;

		if (x_r <= 0.0)
		{
			continue;
		}

		if (distance >= lookahead_distance_)
		{
			target = pose;
			target_distance = distance;
			target_lateral = y_r;
			return true;
		}
	}

	const auto & fallback = path.poses.back();
	const double dx = fallback.pose.position.x - robot_x;
	const double dy = fallback.pose.position.y - robot_y;
	const double distance = std::hypot(dx, dy);
	const double x_r = cos_yaw * dx + sin_yaw * dy;
	const double y_r = -sin_yaw * dx + cos_yaw * dy;

	if (x_r <= 0.0)
	{
		return false;
	}

	target = fallback;
	target_distance = distance;
	target_lateral = y_r;
	return true;
}

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	std::shared_ptr<LocalPlanner> n = std::make_shared<LocalPlanner>();
	rclcpp::spin(n);
	rclcpp::shutdown();
	return 0;
}
