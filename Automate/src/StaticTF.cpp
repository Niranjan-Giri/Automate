#include "Automate/StaticTF.h"

StaticTF::StaticTF() : Node("static_tf")
{
    this->declare_parameter<std::string>("parent_frame",     "base_link");
    this->declare_parameter<std::vector<std::string>>("child_frames", {"Lidar"});

    parent_frame = this->get_parameter("parent_frame").as_string();
    child_frames = this->get_parameter("child_frames").as_string_array();

    if (child_frames.empty()) 
    {
        RCLCPP_ERROR(this->get_logger(), "No child_frames provided! Shutting down.");
        rclcpp::shutdown();
        return;
    }

    for (const auto & frame : child_frames) 
    {
        attempts[frame]    = 0;
        tf_resolved[frame] = false;
        RCLCPP_INFO(this->get_logger(), "Watching frame: %s", frame.c_str());
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&StaticTF::timer_callback, this)
    );
}

void StaticTF::timer_callback()
{
    bool all_exists = true;

    for (const auto& frame : child_frames)
    {
        if (!tf_resolved[frame])
        {
            all_exists = false;
            break;
        }
    }

    if (all_exists)
    {
        RCLCPP_INFO(this->get_logger(), "Every frame is statically transformed with base");
        timer_->cancel();
        return;
    }

    for (const auto& frame : child_frames)
    {
        if (tf_resolved[frame]) continue;
        check_frame(frame);
    }
}

void StaticTF::check_frame(const std::string& frame)
{
    try
    {
        tf_buffer_->lookupTransform(
            parent_frame,
            frame,
            tf2::TimePointZero,
            tf2::durationFromSec(0.05)
        );
        RCLCPP_INFO(
            this->get_logger(),
            "TF exists: %s -> %s", parent_frame.c_str(), frame.c_str()
        );
        tf_resolved[frame] = true;
    }
    catch (tf2::TransformException& e)
    {   
        attempts[frame]++;

        if (attempts[frame] < 2)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "[%s] TF not found (%d/2): %s",
                frame.c_str(), attempts[frame], e.what()
            );
            return;
        }

        RCLCPP_WARN(
            this->get_logger(),
            "[%s] Publishing static identity TF: %s -> %s",
            frame.c_str(), parent_frame.c_str(), frame.c_str()
        );

        publish_static_tf(parent_frame, frame);
        tf_resolved[frame] = true;
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

    t.transform.rotation.x    = 0.0;
    t.transform.rotation.y    = 0.0;
    t.transform.rotation.z    = 0.0;
    t.transform.rotation.w    = 1.0;

    tf_broadcaster_->sendTransform(t);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<StaticTF> n = std::make_shared<StaticTF>();
    rclcpp::spin(n);
    rclcpp::shutdown();
}