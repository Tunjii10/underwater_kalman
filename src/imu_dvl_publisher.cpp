#include <chrono>
#include <fstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ctime>
#include <iomanip>
#include <sstream>

using namespace std::chrono_literals;

class ImuDvlPublisher : public rclcpp::Node
{
public:
    ImuDvlPublisher(const std::string &csv_file)
        : Node("imu_dvl_publisher"), csv_file_(csv_file)
    {
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 50);
        dvl_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("dvl/data", 50);
        ground_truth_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ground_truth", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&ImuDvlPublisher::publishData, this));

        file_.open(csv_file_);
        if (!file_.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Could not open CSV file.");
            std::cout << csv_file_;
            return;
        }

        // Skip the first few lines if they are headers
        for (int i = 0; i < 1; ++i)
        {
            std::string line;
            std::getline(file_, line); // Adjust the number of lines to skip according to your CSV format
        }
    }

private:
    void publishData()
    {
        if (!file_.is_open())
        {
            return;
        }

        std::string line;
        if (std::getline(file_, line))
        {
            std::stringstream ss(line);
            std::string token;
            try
            {
                // Segment 1: IMU data
                std::getline(ss, token, ','); std::string t_imu_str = token;
                std::getline(ss, token, ','); double AccX = std::stod(token);
                std::getline(ss, token, ','); double AccY = std::stod(token);
                std::getline(ss, token, ','); double AccZ = std::stod(token);
                std::getline(ss, token, ','); double GyrX = std::stod(token);
                std::getline(ss, token, ','); double GyrY = std::stod(token);
                std::getline(ss, token, ','); double GyrZ = std::stod(token);
                std::getline(ss, token, ','); double MagX = std::stod(token);
                std::getline(ss, token, ','); double MagY = std::stod(token);
                std::getline(ss, token, ','); double MagZ = std::stod(token);
                std::getline(ss, token, ','); double pressure = std::stod(token);
                std::getline(ss, token, ','); double temperature = std::stod(token);
                std::getline(ss, token, ','); // Skip whitespace

                auto imu_msg = sensor_msgs::msg::Imu();
                imu_msg.header.stamp = parseTimestamp(t_imu_str);
                imu_msg.header.frame_id = "imu_link";
                imu_msg.linear_acceleration.x = AccX;
                imu_msg.linear_acceleration.y = AccY;
                imu_msg.linear_acceleration.z = AccZ;
                imu_msg.angular_velocity.x = GyrX;
                imu_msg.angular_velocity.y = GyrY;
                imu_msg.angular_velocity.z = GyrZ;

                imu_pub_->publish(imu_msg);

                // Segment 2: DVL data
                std::getline(ss, token, ','); std::string t_DVL_str = token;
                std::getline(ss, token, ','); double WX = std::stod(token);
                std::getline(ss, token, ','); double WY = std::stod(token);
                std::getline(ss, token, ','); double WZ = std::stod(token);
                std::getline(ss, token, ','); double BX = std::stod(token);
                std::getline(ss, token, ','); double BY = std::stod(token);
                std::getline(ss, token, ','); double BZ = std::stod(token);
                std::getline(ss, token, ','); // Skip whitespace

                auto dvl_msg = nav_msgs::msg::Odometry();
                dvl_msg.header.stamp = parseTimestamp(t_DVL_str);
                dvl_msg.header.frame_id = "dvl_link";
                dvl_msg.pose.pose.position.x = BX;
                dvl_msg.pose.pose.position.y = BY;
                dvl_msg.pose.pose.position.z = BZ;
                dvl_msg.twist.twist.linear.x = WX;
                dvl_msg.twist.twist.linear.y = WY;
                dvl_msg.twist.twist.linear.z = WZ;

                dvl_pub_->publish(dvl_msg);

                // Segment 3: Filter data
                std::getline(ss, token, ','); std::string t_filter_str = token;
                std::getline(ss, token, ','); double PN = std::stod(token);
                std::getline(ss, token, ','); double PE = std::stod(token);
                std::getline(ss, token, ','); double PD = std::stod(token);

                auto ground_truth_msg = geometry_msgs::msg::PoseStamped();
                ground_truth_msg.header.stamp = parseTimestamp(t_filter_str);
                ground_truth_msg.header.frame_id = "ground_truth_link";
                ground_truth_msg.pose.position.x = PN;
                ground_truth_msg.pose.position.y = PE;
                ground_truth_msg.pose.position.z = PD;

                ground_truth_pub_->publish(ground_truth_msg);
            }
            catch (const std::invalid_argument &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid data format: %s", e.what());
                RCLCPP_ERROR(this->get_logger(), "Line: %s", line.c_str());
            }
            catch (const std::out_of_range &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Data out of range: %s", e.what());
                RCLCPP_ERROR(this->get_logger(), "Line: %s", line.c_str());
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "End of CSV file reached.");
            timer_->cancel();
            file_.close();
        }
    }

    rclcpp::Time parseTimestamp(const std::string &timestamp_str)
    {
        std::tm tm = {};
        std::stringstream ss(timestamp_str);
        ss >> std::get_time(&tm, "%Y/%m/%d/%H:%M:%S");
        // Extract the fractional seconds part
        std::string fractional_seconds_str = timestamp_str.substr(20);
        double fractional_seconds = std::stod(fractional_seconds_str);

        // Convert fractional seconds to nanoseconds
        int64_t nanoseconds = static_cast<int64_t>(fractional_seconds * 1e9);

        // Create the rclcpp::Time object
        return rclcpp::Time(std::mktime(&tm)) + rclcpp::Duration(std::chrono::nanoseconds(nanoseconds));
    }

    std::string csv_file_;
    std::ifstream file_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ground_truth_pub_; // Ground truth publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr dvl_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::string home_dir = std::getenv("HOME");
    std::string relative_path = "/colcon_ws/src/kalman_filters/data/AUV-Alice/AliceData2.csv";
    std::string csv_file_path = home_dir + relative_path;
    rclcpp::spin(std::make_shared<ImuDvlPublisher>(csv_file_path));
    rclcpp::shutdown();
    return 0;
}
