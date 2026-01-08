#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "sl_lidar.h"

#include <signal.h>
#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <unistd.h>   // access()

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
        /* ================= A1 CONFIG ================= */
        const int serial_baudrate = 115200;

        // Try these ports in order (add/remove as you like)
        const std::vector<std::string> port_list = {
            "/dev/rplidar",   // udev alias (recommended)
            "/dev/ttyUSB0",
            "/dev/ttyUSB1",
            "/dev/ttyUSB2",
            "/dev/ttyUSB3"
        };

        const std::string frame_id   = "laser";
        const std::string topic_name = "scan";
        const std::string scan_mode  = "Standard";
        const float scan_frequency   = 10.0f;

        const bool inverted          = false;
        const bool flip_x_axis       = false;
        const bool angle_compensate  = true;
        const float range_min        = 0.15f;

        /* ================= DRIVER ================= */
        drv_ = *createLidarDriver();
        if (!drv_) {
            RCLCPP_ERROR(get_logger(), "Failed to create driver");
            return -1;
        }

        if (!connect_any_port(port_list, serial_baudrate)) {
            RCLCPP_ERROR(get_logger(), "Unable to connect to RPLIDAR on any port");
            cleanup_channel();
            return -1;
        }

        if (!get_device_info() || !check_health()) {
            stop();
            cleanup_channel();
            return -1;
        }

        /* ================= MOTOR (A1 PWM) ================= */
        drv_->setMotorSpeed(600);

        if (!start_scan(scan_mode, scan_frequency)) {
            stop();
            cleanup_channel();
            return -1;
        }

        /* ================= QoS (RViz friendly) ================= */
        scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
            topic_name,
            rclcpp::QoS(rclcpp::KeepLast(10)).reliable()
        );

        RCLCPP_INFO(get_logger(), "Publishing LaserScan on: %s", topic_name.c_str());

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
            } else {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "grabScanDataHq failed (0x%08x), trying to continue...", result);
            }

            rclcpp::spin_some(shared_from_this());
        }

        stop();
        cleanup_channel();
        return 0;
    }

private:
    /* ================= CONNECT WITH FALLBACK ================= */
    bool connect_any_port(const std::vector<std::string>& ports, int baudrate)
    {
        for (const auto& port : ports) {
            // Skip non-existing or non-accessible device nodes
            if (access(port.c_str(), R_OK | W_OK) != 0) {
                RCLCPP_DEBUG(get_logger(), "Port not accessible: %s", port.c_str());
                continue;
            }

            cleanup_channel(); // free previous try (if any)

            channel_ = *createSerialPortChannel(port, baudrate);
            if (!channel_) {
                RCLCPP_WARN(get_logger(), "Failed to create channel for %s", port.c_str());
                continue;
            }

            auto ans = drv_->connect(channel_);
            if (SL_IS_OK(ans)) {
                RCLCPP_INFO(get_logger(), "Connected to RPLIDAR on %s @ %d", port.c_str(), baudrate);
                connected_port_ = port;
                return true;
            }

            RCLCPP_WARN(get_logger(), "Failed to connect on %s (0x%08x)", port.c_str(), ans);
        }
        return false;
    }

    void cleanup_channel()
    {
        // NOTE: sl_lidar channel lifetime depends on SDK version.
        // Many builds require deleting the channel pointer after a failed attempt.
        // If your SDK provides a dispose/release function, use it instead.
        if (channel_) {
            delete channel_;
            channel_ = nullptr;
        }
        connected_port_.clear();
    }

    /* ================= DEVICE ================= */
    bool get_device_info()
    {
        sl_lidar_response_device_info_t info;
        if (SL_IS_FAIL(drv_->getDeviceInfo(info)))
            return false;

        RCLCPP_INFO(
            get_logger(),
            "RPLIDAR A1 | Port %s | FW %d.%02d | HW %d",
            connected_port_.empty() ? "unknown" : connected_port_.c_str(),
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

        if (health.status == SL_LIDAR_STATUS_ERROR) {
            RCLCPP_ERROR(get_logger(), "LIDAR health status ERROR (code=%d)", health.error_code);
            return false;
        }
        return true;
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
            "scan mode: %s | sample rate: %d KHz | max_distance: %.1f m | scan freq: %.1f Hz | compensate x%d",
            selected.scan_mode,
            (int)(1000.0 / selected.us_per_sample + 0.5),
            selected.max_distance,
            freq,
            angle_compensate_multiple_
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
        (void)inverted; // keep params for future use
        (void)flip_x;

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
            msg.ranges[i] = (d > 0) ? d : std::numeric_limits<float>::infinity();
            msg.intensities[i] = nodes[i].quality >> 2;
        }

        scan_pub_->publish(msg);
    }

    void stop()
    {
        if (drv_) {
            drv_->stop();
            drv_->setMotorSpeed(0);
        }
    }

private:
    ILidarDriver* drv_{nullptr};
    IChannel* channel_{nullptr};
    std::string connected_port_;

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
