#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/exceptions.h>

class StaticTF : public rclcpp::Node
{
public:
    StaticTF();

private:
    void sensor_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void publish_static_tf(const std::string& parent_link, const std::string& child_link);

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

    std::string topic_name;
    std::string parent_frame;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl2_sub_;

    bool tf_published;
    int failed_attempt;
};