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
    void timer_callback();
    void check_frame(const std::string& frame);
    void publish_static_tf(const std::string& parent_link, const std::string& child_link);

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

    std::vector<std::string> child_frames;
    std::string parent_frame;

    std::unordered_map<std::string, bool> tf_resolved;
    std::unordered_map<std::string, int> attempts;

    rclcpp::TimerBase::SharedPtr timer_;
};