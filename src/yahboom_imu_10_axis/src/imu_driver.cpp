#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "imu_serial.hpp"   // your C++ IMU driver

using namespace std::chrono_literals;
using ybimu::ImuSerial;

class YbImuDriver : public rclcpp::Node
{
public:
    YbImuDriver()
    : Node("ybimu_node")
    {
        init_topic();
    }

private:
    void init_topic()
    {
        std::vector<std::string> port_list = {
            "/dev/myimu",
            "/dev/ttyUSB0",
            "/dev/ttyUSB1",
            "/dev/ttyUSB2"
        };

        for (const auto &port : port_list)
        {
            try
            {
                imu_ = std::make_shared<ImuSerial>(port);
                imu_->startReceiveThread();
                RCLCPP_INFO(get_logger(), "Open Ybimu Port OK: %s", port.c_str());
                break;
            }
            catch (...)
            {
                // try next port
            }
        }

        if (!imu_)
        {
            RCLCPP_ERROR(get_logger(), "---------Fail To Open Ybimu Serial------------");
            return;
        }

        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", 100);

        // 100 Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&YbImuDriver::pub_data, this)
        );
    }

    void pub_data()
    {
        if (!imu_)
            return;

        auto stamp = this->get_clock()->now();

        auto accel = imu_->getAccel();
        auto gyro  = imu_->getGyro();
        auto quat  = imu_->getQuat();

        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = stamp;
        imu_msg.header.frame_id = "imu_link";

        // Linear acceleration (m/s^2 assumed already scaled correctly)
        imu_msg.linear_acceleration.x = accel[0];
        imu_msg.linear_acceleration.y = accel[1];
        imu_msg.linear_acceleration.z = accel[2];

        // Angular velocity (rad/s)
        imu_msg.angular_velocity.x = gyro[0];
        imu_msg.angular_velocity.y = gyro[1];
        imu_msg.angular_velocity.z = gyro[2];

        // Orientation (quat)
        imu_msg.orientation.w = quat[0];
        imu_msg.orientation.x = quat[1];
        imu_msg.orientation.y = quat[2];
        imu_msg.orientation.z = quat[3];

        imu_pub_->publish(imu_msg);
    }

    std::shared_ptr<ImuSerial> imu_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YbImuDriver>());
    rclcpp::shutdown();
    return 0;
}
