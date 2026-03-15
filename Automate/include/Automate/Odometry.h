#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

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
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void publish_odometry(rclcpp::Time stamp);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl2_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr scan_match_pub_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr previous_pcl2;
    Eigen::Matrix4f pose;

    std::string base_link;
    bool use_imu;
    bool imu_received_ = false;
    Eigen::Quaternionf current_imu_orientation_;
    std::string imu_topic;
};