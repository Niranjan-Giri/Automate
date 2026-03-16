#include "Automate/Odometry.h"

Odometry::Odometry() : Node("odometry")
{
    this->declare_parameter<std::string>("pcl2_topic", "/point_cloud/point_cloud");
    pcl2_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        this->get_parameter("pcl2_topic").as_string(), 10,
        std::bind(&Odometry::pcl2_callback, this, std::placeholders::_1)
    );

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    scan_match_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("scan_match_pose", 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    pose = Eigen::Matrix4f::Identity();

    this->declare_parameter<std::string>("base_link", "base_link");
    base_link = this->get_parameter("base_link").as_string();

    this->declare_parameter<bool>("use_imu", true);
    use_imu = this->get_parameter("use_imu").as_bool();

    this->declare_parameter<std::string>("imu_topic", "/imu");
    imu_topic = this->get_parameter("imu_topic").as_string();

    if (use_imu) {
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 10,
            std::bind(&Odometry::imu_callback, this, std::placeholders::_1)
        );
    }
}

void Odometry::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    current_imu_orientation_ = Eigen::Quaternionf(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    imu_received_ = true;
}

void Odometry::pcl2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pcl2);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*pcl2, *pcl2, indices);

    if (pcl2->size() < 10)
    {
        RCLCPP_WARN(get_logger(), "Point cloud too small: %zu points", pcl2->size());
        return;
    }

    if (!previous_pcl2)
    {
        previous_pcl2 = pcl2;
        return;
    }

    std::vector<int> prev_indices;
    pcl::removeNaNFromPointCloud(*previous_pcl2, *previous_pcl2, prev_indices);

    if (previous_pcl2->size() < 10)
    {
        RCLCPP_WARN(get_logger(), "Previous point cloud too small: %zu points", previous_pcl2->size());
        previous_pcl2 = pcl2;
        return;
    }

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(pcl2);
    icp.setInputTarget(previous_pcl2);

    pcl::PointCloud<pcl::PointXYZ> aligned;
    icp.align(aligned);

    if (!icp.hasConverged())
    {
        RCLCPP_WARN(get_logger(), "ICP did not converge");
        return;
    }

    Eigen::Matrix4f transform = icp.getFinalTransformation();
    pose = pose * transform;

    if (use_imu && imu_received_)
    {
        Eigen::Matrix3f imu_rot = current_imu_orientation_.toRotationMatrix();
        pose.block<3, 3>(0, 0) = imu_rot;
    }

    publish_odometry(msg->header.stamp);
    previous_pcl2 = pcl2;
}

void Odometry::publish_odometry(rclcpp::Time stamp)
{
    nav_msgs::msg::Odometry odom;

    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = base_link;

    odom.pose.pose.position.x = pose(0,3);
    odom.pose.pose.position.y = pose(1,3);
    odom.pose.pose.position.z = pose(2,3);

    Eigen::Matrix3f rot = pose.block<3,3>(0,0);
    Eigen::Quaternionf q(rot);

    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom_pub_->publish(odom);

    geometry_msgs::msg::PoseStamped scan_match_pose;
    scan_match_pose.header.stamp = stamp;
    scan_match_pose.header.frame_id = "map";

    scan_match_pose.pose.position.x = pose(0,3);
    scan_match_pose.pose.position.y = pose(1,3);
    scan_match_pose.pose.position.z = pose(2,3);

    scan_match_pose.pose.orientation.x = q.x();
    scan_match_pose.pose.orientation.y = q.y();
    scan_match_pose.pose.orientation.z = q.z();
    scan_match_pose.pose.orientation.w = q.w();

    scan_match_pub_->publish(scan_match_pose);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = "odom";
    tf.child_frame_id = base_link;

    tf.transform.translation.x = pose(0,3);
    tf.transform.translation.y = pose(1,3);
    tf.transform.translation.z = pose(2,3);

    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<Odometry> n = std::make_shared<Odometry>();
    rclcpp::spin(n);
    rclcpp::shutdown();
}