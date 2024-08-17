#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <rosgraph_msgs/msg/clock.hpp>



using namespace std::chrono_literals;

class ImuDvlPublisher : public rclcpp::Node
{
public:
    ImuDvlPublisher(const std::string &csv_file)
        : Node("imu_dvl_publisher"), csv_file_(csv_file)
    {
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 50);
        dvl_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("dvl/data", 50);
        ground_truth_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ground_truth", 50);
        clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
        
        // Read the CSV file and store the data
        read_csv_file(csv_file_);

        timer_ = this->create_wall_timer(500ms, std::bind(&ImuDvlPublisher::publish_data, this));

        current_row_ = 0;
    }

private:
    void read_csv_file(const std::string &file_path)
    {
        std::ifstream file(file_path);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", file_path.c_str());
            return;
        }

        std::string line;
        std::getline(file, line); // Skip the header

        while (std::getline(file, line))
        {
            std::stringstream ss(line);
            std::string item;
            std::vector<std::string> row;
            // RCLCPP_INFO(this->get_logger(), "Processing line :%zu", line.c_str());
            
            while (std::getline(ss, item, ','))
            {
                row.push_back(item);
            }
            // print_vector(row);
            data_.emplace_back(row);
        }

        file.close();
    }

    void print_vector(const std::vector<std::string>& vec)
    {
        std::string output = "Vector contents: ";
        for (const auto& item : vec)
        {
            output += item + ", ";
        }

        // Remove the trailing comma and space for a cleaner output
        if (!vec.empty())
        {
            output = output.substr(0, output.size() - 2);
        }

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", output.c_str());
    }

    uint64_t convertToNanoseconds(const std::string &timestamp_str)
    {
        std::tm tm = {};
        std::stringstream ss(timestamp_str);

        // Parse the date and time part
        ss >> std::get_time(&tm, "%Y-%m-%d %H:%M:%S");

        // Extract the fractional seconds (microseconds)
        std::string microseconds_str;
        if (ss.peek() == '.')
        {
            ss.ignore(); // Ignore the decimal point
            ss >> microseconds_str;
        }

        // Convert to time since epoch in seconds
        time_t time_since_epoch = timegm(&tm); // gmtime version for time_t

        // Convert seconds to nanoseconds
        uint64_t timestamp_in_nanoseconds = static_cast<uint64_t>(time_since_epoch) * 1e9;

        // Add the fractional seconds in nanoseconds
        if (!microseconds_str.empty())
        {
            uint64_t microseconds = std::stoul(microseconds_str);
            timestamp_in_nanoseconds += microseconds * 1e3; // Convert microseconds to nanoseconds
        }

        return timestamp_in_nanoseconds;
    }

    void publish_data()
    {
        if (current_row_ >= data_.size())
        {
            RCLCPP_INFO(this->get_logger(), "Finished publishing all data");
            rclcpp::shutdown();
            return;
        }

        const auto &row = data_[current_row_];
        try
        {
            // Print the current row for debugging
            // RCLCPP_INFO(this->get_logger(), "Processing row %zu: %s", current_row_, row);
            print_vector(row);

            // Extract timestamp and type
            uint64_t timestamp_ns = convertToNanoseconds(row[0]);
            auto type = row[1];                  // assuming type is the second column

            rclcpp::Time ros_time(timestamp_ns);

            // Publish the clock message
            rosgraph_msgs::msg::Clock clock_msg;
            clock_msg.clock = ros_time;
            clock_pub_->publish(clock_msg);

            // Publish IMU, DVL, or Ground Truth based on the type
            if (type == "IMU")
            {
                auto imu_msg = sensor_msgs::msg::Imu();
                imu_msg.header.stamp = ros_time;
                imu_msg.header.frame_id = "imu_frame";

                // Fill the IMU message fields with error handling
                imu_msg.linear_acceleration.x = std::stod(row[2]);
                imu_msg.linear_acceleration.y = std::stod(row[3]);
                imu_msg.linear_acceleration.z = std::stod(row[4]);

                imu_msg.angular_velocity.x = std::stod(row[5]);
                imu_msg.angular_velocity.y = std::stod(row[6]);
                imu_msg.angular_velocity.z = std::stod(row[7]);

                imu_pub_->publish(imu_msg);
            }
            else if (type == "DVL")
            {
                auto dvl_msg = nav_msgs::msg::Odometry();
                dvl_msg.header.stamp = ros_time;
                dvl_msg.header.frame_id = "dvl_frame";

                // Fill the DVL message fields with error handling
                dvl_msg.twist.twist.linear.x = std::stod(row[13]);
                dvl_msg.twist.twist.linear.y = std::stod(row[14]);
                dvl_msg.twist.twist.linear.z = std::stod(row[15]);

                dvl_pub_->publish(dvl_msg);
            }
            else if (type == "Filter")
            {
                auto ground_truth_msg = geometry_msgs::msg::PoseStamped();
                ground_truth_msg.header.stamp = ros_time;
                ground_truth_msg.header.frame_id = "ground_truth_frame";

                // Fill the ground truth message fields with error handling
                ground_truth_msg.pose.position.x = std::stod(row[19]);
                ground_truth_msg.pose.position.y = std::stod(row[20]);
                ground_truth_msg.pose.position.z = std::stod(row[21]);

                ground_truth_pub_->publish(ground_truth_msg);
            }

            ++current_row_;
        }
        catch (const std::invalid_argument &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid argument encountered: %s", e.what());
            rclcpp::shutdown();
        }
        catch (const std::out_of_range &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Out of range error encountered: %s", e.what());
            rclcpp::shutdown();
        }
    }

    std::string csv_file_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr dvl_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ground_truth_pub_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::vector<std::string>> data_;
    size_t current_row_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::string home_dir = std::getenv("HOME");
    std::string relative_path = "/colcon_ws/src/kalman_filters/data/AUV-Alice/processed_data.csv";
    std::string csv_file_path = home_dir + relative_path;
    rclcpp::spin(std::make_shared<ImuDvlPublisher>(csv_file_path));
    rclcpp::shutdown();
    return 0;
}
