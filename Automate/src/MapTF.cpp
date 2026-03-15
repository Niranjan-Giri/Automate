#include "Automate/MapTF.h"

MapTF::MapTF() : Node("map_tf")
{
    this->declare_parameter<std::string>("odom_topic", "/odom");
    odom_topic = this->get_parameter("odom_topic").as_string();

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10,
        std::bind(&MapTF::odom_callback, this, std::placeholders::_1)
    );

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
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

    tf2::Transform map_pose = odom_pose;

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