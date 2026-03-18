#include "Navigation/LocalCostmap.h"

LocalCostmap::LocalCostmap(): Node("local_costmap")
{
    resolution = 0.05;
    width = 100;
    height = 100;

    origin_x = - (width * resolution) / 2.0;
    origin_y = - (height * resolution) / 2.0;

    cost_data.resize(width * height);

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
    std::fill(cost_data.begin(), cost_data.end(), 0);

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

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

                        int cell = inflated[ny * width + nx];
                        if (cost > cell) 
                        {
                            cell = cost;
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