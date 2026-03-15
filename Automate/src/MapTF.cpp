#include "Automate/MapTF.h"

MapTF::MapTF() : Node("map_tf")
{
    this->declare_parameter<std::string>("odom_topic", "/odom");
    odom_topic = this->get_parameter("odom_topic").as_string();

    this->declare_parameter<std::string>("scan_match_topic", "/scan_match_pose");
    scan_match_topic = this->get_parameter("scan_match_topic").as_string();

    current_map_pose_.setIdentity();

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10,
        std::bind(&MapTF::odom_callback, this, std::placeholders::_1)
    );

    scan_match_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        scan_match_topic, 10,
        std::bind(&MapTF::scan_match_callback, this, std::placeholders::_1)
    );

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void MapTF::scan_match_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    tf2::Transform new_pose;
    new_pose.setOrigin(tf2::Vector3(
        msg->pose.position.x,
        msg->pose.position.y,
        msg->pose.position.z)
    );
    tf2::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w
    );
    new_pose.setRotation(q);

    std::lock_guard<std::mutex> lock(pose_mutex_);
    current_map_pose_ = new_pose;
}

void MapTF::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    tf2::Transform odom_pose;
    odom_pose.setOrigin(tf2::Vector3(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z)
    );

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );

    odom_pose.setRotation(q);

    tf2::Transform map_pose;
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        map_pose = current_map_pose_;
    }

    tf2::Transform map_to_odom = map_pose * odom_pose.inverse();
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = this->get_clock()->now();

    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = msg->header.frame_id;

    tf_msg.transform.translation.x = map_to_odom.getOrigin().x();
    tf_msg.transform.translation.y = map_to_odom.getOrigin().y();
    tf_msg.transform.translation.z = map_to_odom.getOrigin().z();

    tf_msg.transform.rotation.x = map_to_odom.getRotation().x();
    tf_msg.transform.rotation.y = map_to_odom.getRotation().y();
    tf_msg.transform.rotation.z = map_to_odom.getRotation().z();
    tf_msg.transform.rotation.w = map_to_odom.getRotation().w();

    tf_broadcaster_->sendTransform(tf_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<MapTF> n = std::make_shared<MapTF>();
    rclcpp::spin(n);
    rclcpp::shutdown();
    return 0;
}