#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>

#include "sl_lidar.h"
#include <signal.h>
#include <cmath>
#include <limits>

using namespace sl;

#define DEG2RAD(x) ((x) * M_PI / 180.0)

static bool need_exit = false;

class RPlidarA1Node : public rclcpp::Node
{
public:
    RPlidarA1Node()
    : Node("rplidar_a1_node")
    {}

    int run()
    {
        /* ================= FIXED A1 CONFIG ================= */
        const std::string serial_port = "/dev/ttyUSB0";
        const int serial_baudrate = 115200;
        const std::string frame_id = "laser";
        const std::string topic_name = "scan";
        const std::string scan_mode = "Standard";   // change to "Standard" for 10Hz
        const float scan_frequency = 10.0f;

        const bool inverted = false;
        const bool flip_x_axis = false;
        const bool angle_compensate = true;
        const float range_min = 0.15f;

        /* ================= DRIVER ================= */
        drv_ = *createLidarDriver();
        if (!drv_) {
            RCLCPP_ERROR(get_logger(), "Failed to create driver");
            return -1;
        }

        IChannel* channel = *createSerialPortChannel(serial_port, serial_baudrate);
        if (SL_IS_FAIL(drv_->connect(channel))) {
            RCLCPP_ERROR(get_logger(), "Failed to connect to %s", serial_port.c_str());
            return -1;
        }

        if (!get_device_info() || !check_health())
            return -1;

        /* ================= MOTOR (A1 PWM) ================= */
        drv_->setMotorSpeed(600);

        if (!start_scan(scan_mode, scan_frequency))
            return -1;

        /* ================= QoS FIX =================
         * RELIABLE publisher → RViz works immediately
         */
        scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
            topic_name,
            rclcpp::QoS(rclcpp::KeepLast(10)).reliable()
        );

        /* ================= MAIN LOOP ================= */
        while (rclcpp::ok() && !need_exit) {
            sl_lidar_response_measurement_node_hq_t nodes[8192];
            size_t count = sizeof(nodes) / sizeof(nodes[0]);

            rclcpp::Time start = now();
            auto result = drv_->grabScanDataHq(nodes, count);
            rclcpp::Time end = now();

            if (result == SL_RESULT_OK) {
                drv_->ascendScanData(nodes, count);

                float angle_min = DEG2RAD(0.0f);
                float angle_max = DEG2RAD(359.0f);

                if (angle_compensate) {
                    publish_angle_compensated(
                        nodes, count, start,
                        (end - start).seconds(),
                        inverted, flip_x_axis,
                        angle_min, angle_max,
                        frame_id, range_min);
                } else {
                    publish_raw(
                        nodes, count, start,
                        (end - start).seconds(),
                        inverted, flip_x_axis,
                        angle_min, angle_max,
                        frame_id, range_min);
                }
            }

            rclcpp::spin_some(shared_from_this());
        }

        stop();
        return 0;
    }

private:
    /* ================= DEVICE ================= */
    bool get_device_info()
    {
        sl_lidar_response_device_info_t info;
        if (SL_IS_FAIL(drv_->getDeviceInfo(info)))
            return false;

        RCLCPP_INFO(
            get_logger(),
            "RPLIDAR A1 | FW %d.%02d | HW %d",
            info.firmware_version >> 8,
            info.firmware_version & 0xFF,
            info.hardware_version
        );
        return true;
    }

    bool check_health()
    {
        sl_lidar_response_device_health_t health;
        if (SL_IS_FAIL(drv_->getHealth(health)))
            return false;

        return health.status != SL_LIDAR_STATUS_ERROR;
    }

    /* ================= SCAN MODE ================= */
    bool start_scan(const std::string& name, float freq)
    {
        std::vector<LidarScanMode> modes;
        drv_->getAllSupportedScanModes(modes);

        LidarScanMode selected{};
        bool found = false;

        for (auto& m : modes) {
            if (m.scan_mode == name) {
                selected = m;
                found = true;
                break;
            }
        }

        if (!found) {
            RCLCPP_ERROR(get_logger(), "Scan mode %s not supported", name.c_str());
            return false;
        }

        drv_->startScanExpress(false, selected.id, 0, &selected);

        int points_per_circle =
            static_cast<int>(1000000.0 / selected.us_per_sample / freq);

        angle_compensate_multiple_ = points_per_circle / 360 + 1;
        if (angle_compensate_multiple_ < 1)
            angle_compensate_multiple_ = 1;

        max_distance_ = selected.max_distance;

        RCLCPP_INFO(
            get_logger(),
            "current scan mode: %s, sample rate: %d Khz, max_distance: %.1f m, scan frequency:%.1f Hz, ",
            selected.scan_mode,
            (int)(1000.0 / selected.us_per_sample + 0.5),
            selected.max_distance,
            freq
        );

        return true;
    }

    /* ================= PUBLISH ================= */
    static float angle(const sl_lidar_response_measurement_node_hq_t& n)
    {
        return n.angle_z_q14 * 90.f / 16384.f;
    }

    void publish_angle_compensated(
        sl_lidar_response_measurement_node_hq_t* nodes,
        size_t count,
        rclcpp::Time stamp,
        double scan_time,
        bool inverted,
        bool flip_x,
        float angle_min,
        float angle_max,
        const std::string& frame,
        float range_min)
    {
        const int total = 360 * angle_compensate_multiple_;
        std::vector<sl_lidar_response_measurement_node_hq_t> buf(total);

        for (size_t i = 0; i < count; i++) {
            if (nodes[i].dist_mm_q2 == 0) continue;
            int idx = (int)(angle(nodes[i]) * angle_compensate_multiple_);
            for (int j = 0; j < angle_compensate_multiple_; j++) {
                int k = idx + j;
                if (k >= total) k = total - 1;
                buf[k] = nodes[i];
            }
        }

        publish_raw(
            buf.data(), total,
            stamp, scan_time,
            inverted, flip_x,
            angle_min, angle_max,
            frame, range_min);
    }

    void publish_raw(
        sl_lidar_response_measurement_node_hq_t* nodes,
        size_t count,
        rclcpp::Time stamp,
        double scan_time,
        bool inverted,
        bool flip_x,
        float angle_min,
        float angle_max,
        const std::string& frame,
        float range_min)
    {
        sensor_msgs::msg::LaserScan msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = frame;

        msg.angle_min = angle_min;
        msg.angle_max = angle_max;
        msg.angle_increment = (angle_max - angle_min) / (count - 1);
        msg.scan_time = scan_time;
        msg.time_increment = scan_time / (count - 1);
        msg.range_min = range_min;
        msg.range_max = max_distance_;

        msg.ranges.resize(count);
        msg.intensities.resize(count);

        for (size_t i = 0; i < count; i++) {
            float d = nodes[i].dist_mm_q2 / 4.0f / 1000.0f;
            msg.ranges[i] = d > 0 ? d : std::numeric_limits<float>::infinity();
            msg.intensities[i] = nodes[i].quality >> 2;
        }

        scan_pub_->publish(msg);
    }

    void stop()
    {
        drv_->stop();
        drv_->setMotorSpeed(0);
    }

private:
    ILidarDriver* drv_{nullptr};
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

    int angle_compensate_multiple_{1};
    float max_distance_{12.0f};
};

/* ================= SIGNAL ================= */
void sigint_handler(int)
{
    need_exit = true;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    signal(SIGINT, sigint_handler);

    auto node = std::make_shared<RPlidarA1Node>();
    int ret = node->run();

    rclcpp::shutdown();
    return ret;
}
