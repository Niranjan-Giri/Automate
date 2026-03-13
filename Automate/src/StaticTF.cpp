#include "Automate/StaticTF.h"

StaticTF::StaticTF() : Node("static_tf"), tf_published(0), failed_attempt(0)
{
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    this->declare_parameter<std::string>("pcl2_topic", "/point_cloud/point_cloud");
    topic_name = this->get_parameter("pcl2_topic").as_string();

    this->declare_parameter<std::string>("base_link", "base_link");
    parent_frame = this->get_parameter("base_link").as_string();

    pcl2_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic_name, 10,
        std::bind(&StaticTF::sensor_callback, this, std::placeholders::_1)
    );
}

void StaticTF::sensor_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if (tf_published) return;

    const std::string sensor_frame = msg->header.frame_id;

    if (sensor_frame.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Frame name empty for sensor");
        return;
    }

    try
    {
        tf_buffer_->lookupTransform(
            parent_frame,
            sensor_frame,
            tf2::TimePointZero,
            tf2::durationFromSec(0.1)
        );

        RCLCPP_INFO(this->get_logger(), "TF exists: %s -> %s", parent_frame.c_str(), sensor_frame.c_str());
        tf_published = true;
    }
    catch (const tf2::TransformException& e)
    {
        failed_attempt++;

        if (failed_attempt < 2) 
        {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed (%d/2): %s", failed_attempt, e.what());
            return;
        }

        RCLCPP_WARN(this->get_logger(), "No tf for %s -> %s", parent_frame.c_str(), sensor_frame.c_str());
        publish_static_tf(parent_frame, sensor_frame);
    }
}

void StaticTF::publish_static_tf(const std::string& parent_link, const std::string& child_link)
{
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = parent_link;
    t.child_frame_id = child_link;

    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(t);
    tf_published = true;

    RCLCPP_INFO(this->get_logger(), "Published transform %s -> %s", parent_link.c_str(), child_link.c_str());
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<StaticTF> n = std::make_shared<StaticTF>();
    rclcpp::spin(n);
    rclcpp::shutdown();
}