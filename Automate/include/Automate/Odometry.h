#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>

class Odometry : public rclcpp::Node
{
public:
    Odometry();
private:
    void pcl2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void publish_odometry(rclcpp::Time stamp);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl2_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr previous_pcl2;
    Eigen::Matrix4f pose;

    std::string base_link;
};