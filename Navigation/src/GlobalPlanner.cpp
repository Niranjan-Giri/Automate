#include "Navigation/GlobalPlanner.h"

GlobalPlanner::GlobalPlanner() : Node("global_planner")
{
	this->declare_parameter<std::string>("map_topic", "/map");
	this->declare_parameter<std::string>("goal_topic", "/goal_pose");
	this->declare_parameter<std::string>("path_topic", "/global_path");
	this->declare_parameter<std::string>("map_frame", "map");
	this->declare_parameter<std::string>("base_frame", "base_link");
	this->declare_parameter<std::string>("waypoints_file", "");
	this->declare_parameter<bool>("use_waypoints", true);
	this->declare_parameter<int>("occupied_threshold", 50);
	this->declare_parameter<bool>("unknown_is_occupied", true);

	map_topic_ = this->get_parameter("map_topic").as_string();
	goal_topic_ = this->get_parameter("goal_topic").as_string();
	path_topic_ = this->get_parameter("path_topic").as_string();
	map_frame_ = this->get_parameter("map_frame").as_string();
	base_frame_ = this->get_parameter("base_frame").as_string();
	waypoints_file_ = this->get_parameter("waypoints_file").as_string();
	use_waypoints_ = this->get_parameter("use_waypoints").as_bool();
	occupied_threshold_ = this->get_parameter("occupied_threshold").as_int();
	unknown_is_occupied_ = this->get_parameter("unknown_is_occupied").as_bool();
	last_planned_map_stamp_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

	tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

	auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
	map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
		map_topic_,
		map_qos,
		std::bind(&GlobalPlanner::map_callback, this, std::placeholders::_1)
	);

	goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
		goal_topic_,
		10,
		std::bind(&GlobalPlanner::goal_callback, this, std::placeholders::_1)
	);

	auto path_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
	path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic_, path_qos);

	if (use_waypoints_ && !waypoints_file_.empty())
	{
		if (load_waypoints_from_yaml(waypoints_file_, waypoints_))
		{
			RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoint(s) from %s", waypoints_.size(), waypoints_file_.c_str());
		}
		else
		{
			RCLCPP_WARN(this->get_logger(), "Failed to load waypoints from %s", waypoints_file_.c_str());
		}
	}

	RCLCPP_INFO(
		this->get_logger(),
		"Global planner started. map=%s goal=%s path=%s",
		map_topic_.c_str(),
		goal_topic_.c_str(),
		path_topic_.c_str()
	);
}

void GlobalPlanner::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
	{
		std::lock_guard<std::mutex> lock(map_mutex_);
		latest_map_ = msg;
	}

	if (use_waypoints_ && !waypoints_.empty())
	{
		const rclcpp::Time map_stamp = msg->header.stamp;
		if (map_stamp > last_planned_map_stamp_)
		{
			const bool tf_ready = tf_buffer_->canTransform(
				map_frame_,
				base_frame_,
				tf2::TimePointZero,
				tf2::durationFromSec(0.05)
			);
			if (!tf_ready)
			{
				RCLCPP_DEBUG(this->get_logger(), "TF not ready for waypoint planning.");
				return;
			}

			if (plan_waypoints(*msg, waypoints_))
			{
				last_planned_map_stamp_ = map_stamp;
			}
		}
	}
}

void GlobalPlanner::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	if (use_waypoints_ && !waypoints_.empty())
	{
		RCLCPP_DEBUG(this->get_logger(), "Ignoring goal topic because waypoints are enabled.");
		return;
	}

	nav_msgs::msg::OccupancyGrid::SharedPtr map_copy;
	{
		std::lock_guard<std::mutex> lock(map_mutex_);
		map_copy = latest_map_;
	}

	if (!map_copy)
	{
		RCLCPP_WARN(this->get_logger(), "No map received yet. Cannot plan.");
		return;
	}

	double robot_x = 0.0;
	double robot_y = 0.0;
	if (!get_robot_pose(robot_x, robot_y))
	{
		RCLCPP_WARN(this->get_logger(), "Failed to get robot pose (%s -> %s).", map_frame_.c_str(), base_frame_.c_str());
		return;
	}

	int start_gx = 0;
	int start_gy = 0;
	int goal_gx = 0;
	int goal_gy = 0;

	if (!world_to_grid(*map_copy, robot_x, robot_y, start_gx, start_gy))
	{
		RCLCPP_WARN(this->get_logger(), "Robot start is outside map bounds.");
		return;
	}

	if (!world_to_grid(*map_copy, msg->pose.position.x, msg->pose.position.y, goal_gx, goal_gy))
	{
		RCLCPP_WARN(this->get_logger(), "Goal is outside map bounds.");
		return;
	}

	RCLCPP_INFO(
		this->get_logger(),
		"Going to goal: x=%.3f y=%.3f",
		msg->pose.position.x,
		msg->pose.position.y
	);

	if (is_occupied(*map_copy, start_gx, start_gy))
	{
		RCLCPP_WARN(this->get_logger(), "Start cell is occupied. Cannot plan.");
		return;
	}

	if (is_occupied(*map_copy, goal_gx, goal_gy))
	{
		RCLCPP_WARN(this->get_logger(), "Goal cell is occupied. Cannot plan.");
		return;
	}

	std::vector<std::pair<int, int>> cells;
	if (!plan_astar(*map_copy, start_gx, start_gy, goal_gx, goal_gy, cells))
	{
		RCLCPP_WARN(this->get_logger(), "A* failed to find a path.");
		return;
	}

	publish_path(*map_copy, cells);
	RCLCPP_INFO(this->get_logger(), "Published global path with %zu poses.", cells.size());
}

bool GlobalPlanner::load_waypoints_from_yaml(const std::string & path, std::vector<Waypoint> & out) const
{
	try
	{
		YAML::Node root = YAML::LoadFile(path);
		if (!root["waypoints"] || !root["waypoints"].IsSequence())
		{
			RCLCPP_WARN(this->get_logger(), "Waypoints file has no 'waypoints' sequence.");
			return false;
		}

		out.clear();
		for (const auto & wp : root["waypoints"])
		{
			if (!wp["x"] || !wp["y"])
			{
				RCLCPP_WARN(this->get_logger(), "Skipping waypoint missing x or y.");
				continue;
			}
			Waypoint point;
			point.x = wp["x"].as<double>();
			point.y = wp["y"].as<double>();
			out.push_back(point);
		}

		return !out.empty();
	}
	catch (const YAML::Exception & e)
	{
		RCLCPP_ERROR(this->get_logger(), "YAML parse error: %s", e.what());
		return false;
	}
}

bool GlobalPlanner::plan_waypoints(const nav_msgs::msg::OccupancyGrid & grid, const std::vector<Waypoint> & waypoints)
{
	if (waypoints.empty())
	{
		RCLCPP_WARN(this->get_logger(), "No waypoints provided.");
		return false;
	}

	double robot_x = 0.0;
	double robot_y = 0.0;
	if (!get_robot_pose(robot_x, robot_y))
	{
		RCLCPP_WARN(this->get_logger(), "Failed to get robot pose (%s -> %s).", map_frame_.c_str(), base_frame_.c_str());
		return false;
	}

	int start_gx = 0;
	int start_gy = 0;
	if (!world_to_grid(grid, robot_x, robot_y, start_gx, start_gy))
	{
		RCLCPP_WARN(this->get_logger(), "Robot start is outside map bounds.");
		return false;
	}

	if (is_occupied(grid, start_gx, start_gy))
	{
		RCLCPP_WARN(this->get_logger(), "Start cell is occupied. Cannot plan.");
		return false;
	}

	std::vector<std::pair<int, int>> combined_cells;
	int current_gx = start_gx;
	int current_gy = start_gy;

	for (size_t i = 0; i < waypoints.size(); ++i)
	{
		int goal_gx = 0;
		int goal_gy = 0;
		if (!world_to_grid(grid, waypoints[i].x, waypoints[i].y, goal_gx, goal_gy))
		{
			RCLCPP_WARN(this->get_logger(), "Waypoint %zu is outside map bounds.", i);
			return false;
		}

		RCLCPP_INFO(
			this->get_logger(),
			"Going to waypoint %zu: x=%.3f y=%.3f",
			i,
			waypoints[i].x,
			waypoints[i].y
		);

		if (is_occupied(grid, goal_gx, goal_gy))
		{
			RCLCPP_WARN(this->get_logger(), "Waypoint %zu is occupied. Cannot plan.", i);
			return false;
		}

		std::vector<std::pair<int, int>> segment;
		if (!plan_astar(grid, current_gx, current_gy, goal_gx, goal_gy, segment))
		{
			RCLCPP_WARN(this->get_logger(), "A* failed to reach waypoint %zu.", i);
			return false;
		}

		// Skip the first cell on subsequent segments to avoid duplicate joins.
		const size_t start_idx = combined_cells.empty() ? 0 : 1;
		combined_cells.insert(combined_cells.end(), segment.begin() + start_idx, segment.end());
		current_gx = goal_gx;
		current_gy = goal_gy;
	}

	publish_path(grid, combined_cells);
	RCLCPP_INFO(this->get_logger(), "Published waypoint path with %zu poses.", combined_cells.size());
	return true;
}

bool GlobalPlanner::get_robot_pose(double & x, double & y)
{
	try
	{
		geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
			map_frame_,
			base_frame_,
			tf2::TimePointZero,
			tf2::durationFromSec(0.2)
		);

		x = tf.transform.translation.x;
		y = tf.transform.translation.y;
		return true;
	}
	catch (const tf2::TransformException & e)
	{
		RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", e.what());
		return false;
	}
}

bool GlobalPlanner::world_to_grid(const nav_msgs::msg::OccupancyGrid & grid, double wx, double wy, int & gx, int & gy) const
{
	const double origin_x = grid.info.origin.position.x;
	const double origin_y = grid.info.origin.position.y;
	const double resolution = grid.info.resolution;

	gx = static_cast<int>((wx - origin_x) / resolution);
	gy = static_cast<int>((wy - origin_y) / resolution);

	return in_bounds(grid, gx, gy);
}

void GlobalPlanner::grid_to_world(const nav_msgs::msg::OccupancyGrid & grid, int gx, int gy, double & wx, double & wy) const
{
	const double origin_x = grid.info.origin.position.x;
	const double origin_y = grid.info.origin.position.y;
	const double resolution = grid.info.resolution;

	wx = (static_cast<double>(gx) + 0.5) * resolution + origin_x;
	wy = (static_cast<double>(gy) + 0.5) * resolution + origin_y;
}

bool GlobalPlanner::in_bounds(const nav_msgs::msg::OccupancyGrid & grid, int gx, int gy) const
{
	return gx >= 0 &&
		   gy >= 0 &&
		   gx < static_cast<int>(grid.info.width) &&
		   gy < static_cast<int>(grid.info.height);
}

int GlobalPlanner::flat_index(const nav_msgs::msg::OccupancyGrid & grid, int gx, int gy) const
{
	return gy * static_cast<int>(grid.info.width) + gx;
}

bool GlobalPlanner::is_occupied(const nav_msgs::msg::OccupancyGrid & grid, int gx, int gy) const
{
	if (!in_bounds(grid, gx, gy))
	{
		return true;
	}

	const int value = grid.data[flat_index(grid, gx, gy)];
	if (value < 0)
	{
		return unknown_is_occupied_;
	}

	return value > occupied_threshold_;
}

bool GlobalPlanner::plan_astar(
	const nav_msgs::msg::OccupancyGrid & grid,
	int sx,
	int sy,
	int gx,
	int gy,
	std::vector<std::pair<int, int>> & out_cells
) const
{
	const int width = static_cast<int>(grid.info.width);
	const int height = static_cast<int>(grid.info.height);
	const int total = width * height;

	const int start_idx = flat_index(grid, sx, sy);
	const int goal_idx = flat_index(grid, gx, gy);

	std::vector<float> g_score(total, std::numeric_limits<float>::infinity());
	std::vector<int> parent(total, -1);
	std::vector<bool> closed(total, false);

	auto heuristic = [gx, gy](int x, int y) {
		return static_cast<float>(std::hypot(static_cast<float>(x - gx), static_cast<float>(y - gy)));
	};

	std::priority_queue<OpenEntry, std::vector<OpenEntry>, std::greater<OpenEntry>> open_set;
	g_score[start_idx] = 0.0f;
	open_set.push(OpenEntry{heuristic(sx, sy), start_idx});

	const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
	const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

	while (!open_set.empty())
	{
		const OpenEntry current = open_set.top();
		open_set.pop();

		const int current_idx = current.idx;
		if (closed[current_idx])
		{
			continue;
		}
		closed[current_idx] = true;

		if (current_idx == goal_idx)
		{
			out_cells.clear();
			int walk = goal_idx;
			while (walk != -1)
			{
				const int x = walk % width;
				const int y = walk / width;
				out_cells.emplace_back(x, y);
				if (walk == start_idx)
				{
					break;
				}
				walk = parent[walk];
			}
			std::reverse(out_cells.begin(), out_cells.end());
			return true;
		}

		const int cx = current_idx % width;
		const int cy = current_idx / width;

		for (int i = 0; i < 8; ++i)
		{
			const int nx = cx + dx[i];
			const int ny = cy + dy[i];

			if (!in_bounds(grid, nx, ny) || is_occupied(grid, nx, ny))
			{
				continue;
			}

			const int nidx = flat_index(grid, nx, ny);
			if (closed[nidx])
			{
				continue;
			}

			const bool diagonal = (dx[i] != 0 && dy[i] != 0);
			const float step_cost = diagonal ? 1.4142135f : 1.0f;
			const float tentative_g = g_score[current_idx] + step_cost;

			if (tentative_g < g_score[nidx])
			{
				g_score[nidx] = tentative_g;
				parent[nidx] = current_idx;
				const float f = tentative_g + heuristic(nx, ny);
				open_set.push(OpenEntry{f, nidx});
			}
		}
	}

	return false;
}

void GlobalPlanner::publish_path(const nav_msgs::msg::OccupancyGrid & grid, const std::vector<std::pair<int, int>> & cells)
{
	nav_msgs::msg::Path path_msg;
	path_msg.header.stamp = this->get_clock()->now();
	path_msg.header.frame_id = map_frame_;

	path_msg.poses.reserve(cells.size());
	for (const auto & cell : cells)
	{
		geometry_msgs::msg::PoseStamped pose;
		pose.header = path_msg.header;

		double wx = 0.0;
		double wy = 0.0;
		grid_to_world(grid, cell.first, cell.second, wx, wy);

		pose.pose.position.x = wx;
		pose.pose.position.y = wy;
		pose.pose.position.z = 0.0;
		pose.pose.orientation.w = 1.0;
		path_msg.poses.push_back(pose);
	}

	path_pub_->publish(path_msg);
}

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	std::shared_ptr<GlobalPlanner> n = std::make_shared<GlobalPlanner>();
	rclcpp::spin(n);
	rclcpp::shutdown();
	return 0;
}