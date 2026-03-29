#include "Navigation/OccupancyGrid.h"

// Log-odds constants - for probabilistic approach
static constexpr float L_OCC  =  0.85f;
static constexpr float L_FREE = -0.4f;
static constexpr float L_MIN  = -2.0f;
static constexpr float L_MAX  =  3.5f;

OccupancyGrid::OccupancyGrid() : Node("occupancy_grid")
{
    this->declare_parameter<std::string>("pcl2_topic", "/point_cloud/point_cloud");
    auto sensor_qos = rclcpp::SensorDataQoS();
    pcl2_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        this->get_parameter("pcl2_topic").as_string(), sensor_qos,
        std::bind(&OccupancyGrid::pcl2_callback, this, std::placeholders::_1)
    );

    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<double>("resolution", 0.1);
    this->declare_parameter<double>("min_height", 0.001);
    this->declare_parameter<double>("max_height", 5.0);
    this->declare_parameter<double>("unknown_threshold", 0.01);

    map_frame = this->get_parameter("map_frame").as_string();
    resolution = this->get_parameter("resolution").as_double();
    min_height = this->get_parameter("min_height").as_double();
    max_height = this->get_parameter("max_height").as_double();
    unknown_threshold = this->get_parameter("unknown_threshold").as_double();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(
        this->get_clock(),
        tf2::Duration(std::chrono::seconds(30))
    );
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    grid_.header.frame_id = map_frame;
    grid_.info.resolution = resolution;
    grid_.info.width = 1000;
    grid_.info.height = 1000;

    grid_.info.origin.position.x = - (grid_.info.width / 2.0) * resolution;
    grid_.info.origin.position.y = - (grid_.info.height / 2.0) * resolution;
    grid_.info.origin.position.z = 0.0;
    grid_.info.origin.orientation.w = 1.0;

    int size = grid_.info.width * grid_.info.height;
    grid_.data.assign(size, -1);
    
    log_odds.assign(size, 0.0f);

    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", map_qos);
}

void OccupancyGrid::update_cell(int index, float delta)
{
    log_odds[index] = std::clamp(log_odds[index] + delta, L_MIN, L_MAX);
}

void OccupancyGrid::log_odds_to_grid()
{
    for (size_t i = 0; i < log_odds.size(); i++)
    {
        if (std::fabs(log_odds[i]) <= static_cast<float>(unknown_threshold))
        {
            grid_.data[i] = -1;
        }
        else
        {
            float probability = 1.0f - (1.0f / (1.0f + std::exp(log_odds[i])));
            grid_.data[i] = static_cast<int8_t>(std::clamp(probability * 100.0f, 0.f, 100.0f));
        }
    }
}

void OccupancyGrid::pcl2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    geometry_msgs::msg::TransformStamped transform;
    try 
    {
        transform = tf_buffer_->lookupTransform(
            map_frame,
            msg->header.frame_id,
            tf2::TimePointZero,
            tf2::durationFromSec(1.0)
        );
    } 
    catch (tf2::TransformException & e) 
    {
        RCLCPP_WARN(this->get_logger(), "Transform failed: %s", e.what());
        return;
    }

    float sensor_map_x = transform.transform.translation.x;
    float sensor_map_y = transform.transform.translation.y;

    int sensor_gx = static_cast<int>((sensor_map_x - grid_.info.origin.position.x) / resolution);
    int sensor_gy = static_cast<int>((sensor_map_y - grid_.info.origin.position.y) / resolution);

    sensor_msgs::msg::PointCloud2 cloud_transformed;
    tf2::doTransform(*msg, cloud_transformed, transform);

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(cloud_transformed, pcl_cloud);

    mark_free_circle(sensor_gx, sensor_gy, 3);

    for (const auto & point : pcl_cloud.points)
    {
        if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z))
            continue;

        //if (point.z < min_height || point.z > max_height)
        //    continue;


        int x_idx = static_cast<int>((point.x - grid_.info.origin.position.x) / resolution);
        int y_idx = static_cast<int>((point.y - grid_.info.origin.position.y) / resolution);

        raycast(sensor_gx, sensor_gy, x_idx, y_idx);

        if (x_idx >= 0 && x_idx < static_cast<int>(grid_.info.width) &&
            y_idx >= 0 && y_idx < static_cast<int>(grid_.info.height))
        {
            update_cell(y_idx * grid_.info.width + x_idx, L_OCC);
        }
    }

    log_odds_to_grid();

    grid_.header.stamp = this->get_clock()->now();
    grid_.header.frame_id = map_frame;
    map_pub_->publish(grid_);
}

void OccupancyGrid::raycast(int x0, int y0, int x1, int y1)
{
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = (dx > dy ? dx : -dy) / 2;

    while (true)
    {
        if (x0 == x1 && y0 == y1) break;

        if (x0 < 0 || x0 >= static_cast<int>(grid_.info.width) ||
            y0 < 0 || y0 >= static_cast<int>(grid_.info.height))
            break;

        update_cell(y0 * grid_.info.width + x0, L_FREE);

        int e2 = err;
        if (e2 > -dx) { err -= dy; x0 += sx; }
        if (e2 <  dy) { err += dx; y0 += sy; }
    }
}

void OccupancyGrid::mark_free_circle(int cx, int cy, int radius)
{
    // Marks a small circle of cells as free around a point
    // Handles the "white blob at sensor origin" problem
    for (int dy = -radius; dy <= radius; dy++) 
    {
        for (int dx = -radius; dx <= radius; dx++) 
        {
            if (dx*dx + dy*dy > radius*radius) continue;

            int x = cx + dx;
            int y = cy + dy;

            if (x < 0 || x >= static_cast<int>(grid_.info.width) ||
                y < 0 || y >= static_cast<int>(grid_.info.height))
                continue;

            update_cell(y * grid_.info.width + x, L_FREE);
        }
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<OccupancyGrid> n = std::make_shared<OccupancyGrid>();
    rclcpp::spin(n);
    rclcpp::shutdown();
}