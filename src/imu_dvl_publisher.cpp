#include <chrono>
#include <fstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

class ImuDvlPublisher : public rclcpp::Node
{
public:
    ImuDvlPublisher(const std::string &csv_file)
        : Node("imu_dvl_publisher"), csv_file_(csv_file)
    {
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
        dvl_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("dvl/data", 10);
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

            // Assuming CSV format: timestamp, imu_x, imu_y, imu_z, imu_w, dvl_x, dvl_y, dvl_z, dvl_vx, dvl_vy, dvl_vz
            double timestamp, imu_x, imu_y, imu_z, imu_w, dvl_x, dvl_y, dvl_z, dvl_vx, dvl_vy, dvl_vz;
            try {
                std::getline(ss, token, ','); timestamp = std::stod(token);
                std::getline(ss, token, ','); imu_x = std::stod(token);
                std::getline(ss, token, ','); imu_y = std::stod(token);
                std::getline(ss, token, ','); imu_z = std::stod(token);
                std::getline(ss, token, ','); imu_w = std::stod(token);
                std::getline(ss, token, ','); dvl_x = std::stod(token);
                std::getline(ss, token, ','); dvl_y = std::stod(token);
                std::getline(ss, token, ','); dvl_z = std::stod(token);
                std::getline(ss, token, ','); dvl_vx = std::stod(token);
                std::getline(ss, token, ','); dvl_vy = std::stod(token);
                std::getline(ss, token, ','); dvl_vz = std::stod(token);

                imu_data_.emplace_back(timestamp, imu_x, imu_y, imu_z, imu_w);
                dvl_data_.emplace_back(timestamp, dvl_x, dvl_y, dvl_z, dvl_vx, dvl_vy, dvl_vz);
            }    
            catch (const std::invalid_argument &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid data format: %s", e.what());
                continue; // Skip this line and proceed to the next one
            } 
            catch (const std::out_of_range &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Data out of range: %s", e.what());
                continue; // Skip this line and proceed to the next one
            }
        }
    }

    void publishData()
    {
        if (current_index_ >= imu_data_.size())
        {
            return;
        }

        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = this->get_clock()->now();
        imu_msg.header.frame_id = "imu_link";
        imu_msg.orientation.x = std::get<1>(imu_data_[current_index_]);
        imu_msg.orientation.y = std::get<2>(imu_data_[current_index_]);
        imu_msg.orientation.z = std::get<3>(imu_data_[current_index_]);
        imu_msg.orientation.w = std::get<4>(imu_data_[current_index_]);

        imu_pub_->publish(imu_msg);

        auto dvl_msg = nav_msgs::msg::Odometry();
        dvl_msg.header.stamp = this->get_clock()->now();
        dvl_msg.header.frame_id = "dvl_link";
        dvl_msg.pose.pose.position.x = std::get<1>(dvl_data_[current_index_]);
        dvl_msg.pose.pose.position.y = std::get<2>(dvl_data_[current_index_]);
        dvl_msg.pose.pose.position.z = std::get<3>(dvl_data_[current_index_]);
        dvl_msg.twist.twist.linear.x = std::get<4>(dvl_data_[current_index_]);
        dvl_msg.twist.twist.linear.y = std::get<5>(dvl_data_[current_index_]);
        dvl_msg.twist.twist.linear.z = std::get<6>(dvl_data_[current_index_]);

        dvl_pub_->publish(dvl_msg);

        current_index_++;
    }

    std::string csv_file_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr dvl_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::tuple<double, double, double, double, double>> imu_data_;
    std::vector<std::tuple<double, double, double, double, double, double, double>> dvl_data_;
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
