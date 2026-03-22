#include "Navigation/LocalCostmap.h"

LocalCostmap::LocalCostmap(): Node("local_costmap")
{
    resolution = 0.05;
    width = 100;
    height = 100;

    origin_x = - (width * resolution) / 2.0;
    origin_y = - (height * resolution) / 2.0;

    cost_data.resize(width * height);

    this->declare_parameter<bool>("map_frame", true);
    map_frame = this->get_parameter("map_frame").as_bool();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    this->declare_parameter<std::string>("pcl2_topic", "/lidar/point_cloud");
    pcl2_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        this->get_parameter("pcl2_topic").as_string(), 10,
        std::bind(&LocalCostmap::pointcloud_callback, this, std::placeholders::_1)
    );

    grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/local_costmap", 10);
    RCLCPP_INFO(this->get_logger(), "Local costmap running!");
}

void LocalCostmap::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    sensor_msgs::msg::PointCloud2 cloud_map;

    if (map_frame)
    {
        try 
        {
            tf_buffer_->transform(
                *msg,
                cloud_map,
                "map",
                tf2::durationFromSec(0.1)
            );
        } 
        catch (tf2::TransformException &e) 
        {
            RCLCPP_WARN(this->get_logger(), "TF failed: %s", e.what());
            return;
        }
    }

    std::fill(cost_data.begin(), cost_data.end(), 0);

    pcl::PointCloud<pcl::PointXYZ> cloud;

    if (map_frame)
        pcl::fromROSMsg(cloud_map, cloud);
    else
        pcl::fromROSMsg(*msg, cloud);


    geometry_msgs::msg::TransformStamped tf;

    try 
    {
        tf = tf_buffer_->lookupTransform(
            "map", "base_link", tf2::TimePointZero
        );
    } 
    catch (...) 
    {
        return;
    }

    double robot_x = tf.transform.translation.x;
    double robot_y = tf.transform.translation.y;

    if (map_frame)
    {
        origin_x = robot_x - (width * resolution)/2;
        origin_y = robot_y - (height * resolution)/2;
    }

    for (const auto& point : cloud.points)
    {
        int gx = (point.x - origin_x) / resolution;
        int gy = (point.y - origin_y) / resolution;

        if (gx >= 0 && gx < width && gy >= 0 && gy < height)
            cost_data[gy * width + gx] = 100;
    }

    inflate_obstacles();
    publish_costmap(msg->header.stamp);
}

void LocalCostmap::inflate_obstacles()
{
    std::vector<int8_t> inflated = cost_data;

    int radius = 3;
    float decay = 20.0;

    for (int y = 0; y < height; y++) 
    {
        for (int x = 0; x < width; x++) 
        {
            if (cost_data[y * width + x] == 100) 
            {
                for (int dy = -radius; dy <= radius; dy++) 
                {
                    for (int dx = -radius; dx <= radius; dx++) 
                    {
                        int nx = x + dx;
                        int ny = y + dy;

                        if (nx < 0 || nx >= width || ny < 0 || ny >= height)
                            continue;

                        float dist = std::sqrt(dx*dx + dy*dy);
                        if (dist > radius) continue;

                        int cost = 100 - (dist * decay);
                        cost = std::max(cost, 0);

                        int* cell = new int; 
                        *cell = inflated[ny * width + nx];
                        if (cost > *cell)
                        {
                            *cell = cost;
                        }
                    }
                }
            }
        }
    }

    cost_data = inflated;
}

void LocalCostmap::publish_costmap(rclcpp::Time stamp)
{
    nav_msgs::msg::OccupancyGrid grid;

    grid.header.stamp = stamp;

    if (map_frame)
        grid.header.frame_id = "map";
    else
        grid.header.frame_id = "base_link";

    grid.info.resolution = resolution;

    grid.info.width = width;
    grid.info.height = height;
    
    grid.info.origin.position.x = origin_x;
    grid.info.origin.position.y = origin_y;
    grid.info.origin.position.z = 0.0;

    grid.data = cost_data;
    grid_pub_->publish(grid);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<LocalCostmap> n = std::make_shared<LocalCostmap>();
    rclcpp::spin(n);
    rclcpp::shutdown();
}