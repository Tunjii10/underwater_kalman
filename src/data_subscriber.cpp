#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class DataSubscriber : public rclcpp::Node
{
public:
    DataSubscriber()
        : Node("data_subscriber")
    {
        // Subscribe to IMU data
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", 10,
            std::bind(&DataSubscriber::imu_callback, this, std::placeholders::_1));

        // Subscribe to DVL data
        dvl_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "dvl/data", 10,
            std::bind(&DataSubscriber::dvl_callback, this, std::placeholders::_1));

        // Subscribe to Ground Truth data
        ground_truth_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "ground_truth", 10,
            std::bind(&DataSubscriber::ground_truth_callback, this, std::placeholders::_1));
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "IMU Data received:");
        RCLCPP_INFO(this->get_logger(), "  Timestamp: %d.%d",
                    msg->header.stamp.sec, msg->header.stamp.nanosec);
        RCLCPP_INFO(this->get_logger(), "  Linear Acceleration: [%.2f, %.2f, %.2f]",
                    msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        RCLCPP_INFO(this->get_logger(), "  Angular Velocity: [%.2f, %.2f, %.2f]",
                    msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        // Note: Orientation data is not used here; you can print if needed
    }

    void dvl_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "DVL Data received:");
        RCLCPP_INFO(this->get_logger(), "  Timestamp: %d.%d",
                    msg->header.stamp.sec, msg->header.stamp.nanosec);
        RCLCPP_INFO(this->get_logger(), "  Position: [%.2f, %.2f, %.2f]",
                    msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        RCLCPP_INFO(this->get_logger(), "  Linear Velocity: [%.2f, %.2f, %.2f]",
                    msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    }

    void ground_truth_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Ground Truth Data received:");
        RCLCPP_INFO(this->get_logger(), "  Timestamp: %d.%d",
                    msg->header.stamp.sec, msg->header.stamp.nanosec);
        RCLCPP_INFO(this->get_logger(), "  Position: [%.2f, %.2f, %.2f]",
                    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr dvl_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ground_truth_subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DataSubscriber>());
    rclcpp::shutdown();
    return 0;
}
