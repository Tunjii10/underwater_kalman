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
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
        dvl_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("dvl/data", 10);
        ground_truth_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ground_truth", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&ImuDvlPublisher::publishData, this));
        readCsvFile();
    }

private:
    void readCsvFile()
    {
        std::ifstream file(csv_file_);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Could not open CSV file.");
            std::cout<<csv_file_;
            return;
        }

        std::string line;
        // Skip the first few lines if they are headers
        for (int i = 0; i < 1; ++i)
        {
            std::getline(file, line); // Adjust the number of lines to skip according to your CSV format
        }
        while (std::getline(file, line))
        {
            std::stringstream ss(line);
            std::string token;

            try {
                // Segment 1: IMU data
                std::getline(ss, token, '\t'); std::string t_imu_str = token;
                std::getline(ss, token, '\t'); double AccX = std::stod(token);
                std::getline(ss, token, '\t'); double AccY = std::stod(token);
                std::getline(ss, token, '\t'); double AccZ = std::stod(token);
                std::getline(ss, token, '\t'); double GyrX = std::stod(token);
                std::getline(ss, token, '\t'); double GyrY = std::stod(token);
                std::getline(ss, token, '\t'); double GyrZ = std::stod(token);
                std::getline(ss, token, '\t'); double MagX = std::stod(token);
                std::getline(ss, token, '\t'); double MagY = std::stod(token);
                std::getline(ss, token, '\t'); double MagZ = std::stod(token);
                std::getline(ss, token, '\t'); double pressure = std::stod(token);
                std::getline(ss, token, '\t'); double temperature = std::stod(token);
                std::getline(ss, token, '\t'); // Skip whitespace

                imu_data_.emplace_back(t_imu_str, AccX, AccY, AccZ, GyrX, GyrY, GyrZ, MagX, MagY, MagZ, pressure, temperature);

                // Segment 2: DVL data
                std::getline(ss, token, '\t'); std::string t_DVL_str = token;
                std::getline(ss, token, '\t'); double WX = std::stod(token);
                std::getline(ss, token, '\t'); double WY = std::stod(token);
                std::getline(ss, token, '\t'); double WZ = std::stod(token);
                std::getline(ss, token, '\t'); double BX = std::stod(token);
                std::getline(ss, token, '\t'); double BY = std::stod(token);
                std::getline(ss, token, '\t'); double BZ = std::stod(token);
                std::getline(ss, token, '\t'); // Skip whitespace

                dvl_data_.emplace_back(t_DVL_str, WX, WY, WZ, BX, BY, BZ);

                // Segment 3: Filter data
                std::getline(ss, token, '\t'); std::string t_filter_str = token;
                std::getline(ss, token, '\t'); double PN = std::stod(token);
                std::getline(ss, token, '\t'); double PE = std::stod(token);
                std::getline(ss, token, '\t'); double PD = std::stod(token);

                filter_data_.emplace_back(t_filter_str, PN, PE, PD);
            }    
            catch (const std::invalid_argument &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid data format: %s", e.what());
                RCLCPP_ERROR(this->get_logger(), "Line: %s", line.c_str());
                continue; // Skip this line and proceed to the next one
            } 
            catch (const std::out_of_range &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Data out of range: %s", e.what());
                RCLCPP_ERROR(this->get_logger(), "Line: %s", line.c_str());
                continue; // Skip this line and proceed to the next one
            }
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

    void publishData()
    {
        if (current_index_ >= imu_data_.size() || current_index_ >= dvl_data_.size() || current_index_ >= filter_data_.size())
        {
            return;
        }

        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = parseTimestamp(std::get<0>(imu_data_[current_index_]));
        imu_msg.header.frame_id = "imu_link";
        imu_msg.linear_acceleration.x = std::get<1>(imu_data_[current_index_]);
        imu_msg.linear_acceleration.y = std::get<2>(imu_data_[current_index_]);
        imu_msg.linear_acceleration.z = std::get<3>(imu_data_[current_index_]);
        imu_msg.angular_velocity.x = std::get<4>(imu_data_[current_index_]);
        imu_msg.angular_velocity.y = std::get<5>(imu_data_[current_index_]);
        imu_msg.angular_velocity.z = std::get<6>(imu_data_[current_index_]);
        // imu_msg.orientation_covariance[0] = -1; // Orientation not provided

        imu_pub_->publish(imu_msg);

        auto dvl_msg = nav_msgs::msg::Odometry();
        dvl_msg.header.stamp = parseTimestamp(std::get<0>(dvl_data_[current_index_]));
        dvl_msg.header.frame_id = "dvl_link";
        dvl_msg.pose.pose.position.x = std::get<1>(dvl_data_[current_index_]);
        dvl_msg.pose.pose.position.y = std::get<2>(dvl_data_[current_index_]);
        dvl_msg.pose.pose.position.z = std::get<3>(dvl_data_[current_index_]);
        dvl_msg.twist.twist.linear.x = std::get<4>(dvl_data_[current_index_]);
        dvl_msg.twist.twist.linear.y = std::get<5>(dvl_data_[current_index_]);
        dvl_msg.twist.twist.linear.z = std::get<6>(dvl_data_[current_index_]);

        dvl_pub_->publish(dvl_msg);

        // Publish Ground Truth data
        auto ground_truth_msg = geometry_msgs::msg::PoseStamped();
        ground_truth_msg.header.stamp = parseTimestamp(std::get<0>(filter_data_[current_index_]));
        ground_truth_msg.header.frame_id = "ground_truth_link";
        ground_truth_msg.pose.position.x = std::get<1>(filter_data_[current_index_]);
        ground_truth_msg.pose.position.y = std::get<2>(filter_data_[current_index_]);
        ground_truth_msg.pose.position.z = std::get<3>(filter_data_[current_index_]);

        ground_truth_pub_->publish(ground_truth_msg);

        current_index_++;
    }

    std::string csv_file_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ground_truth_pub_; // Ground truth publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr dvl_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::tuple<std::string, double, double, double, double, double, double, double, double, double, double, double>> imu_data_;
    std::vector<std::tuple<std::string, double, double, double, double, double, double>> dvl_data_;
    std::vector<std::tuple<std::string, double, double, double>> filter_data_;
    size_t current_index_ = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::string home_dir = std::getenv("HOME");
    std::string relative_path = "/colcon_ws/src/kalman_filters/data/AUV-Alice/AliceData1.csv";
    std::string csv_file_path = home_dir + relative_path;
    rclcpp::spin(std::make_shared<ImuDvlPublisher>(csv_file_path));
    rclcpp::shutdown();
    return 0;
}
