#ifndef WHEELS_DRIVER_HPP
#define WHEELS_DRIVER_HPP

#include "food_del_robot_hardware/zlac8015d_driver.hpp"
#include <memory>
#include <string>
#include <cstdint>

class WheelsDriver {
public:
    // Constructor with CAN parameters
    explicit WheelsDriver(const std::string& can_interface, int bitrate, uint8_t device_id);
    ~WheelsDriver();
    
    // Initialize CAN and create driver
    bool init();
    
    // Mode & state
    bool set_velocity_mode();
    bool enable();
    bool emergency_stop();
    bool status_emergency_stop();
    bool release_emergency_stop();
    bool status_release_emergency_stop();
    bool shutdown();
    
    // Set commands (input in rad/s)
    bool set_sync_left_right_speed(float left_rad, float right_rad);

    // Read commands (output in rad/s)
    bool get_speed_rad_per_sec(float& left_rad, float& right_rad);
    
private:
    std::unique_ptr<CAN> can_interface_;
    std::unique_ptr<ZLAC8015DDriver> zlac_driver_;
    
    std::string can_interface_name_;
    int can_bitrate_;
    uint8_t device_id_;
    
    // Conversion constants
    static constexpr float RPM_TO_RAD_PER_SEC = 0.104719755f;  // 2π/60
    static constexpr float RAD_PER_SEC_TO_RPM = 9.5492965855f; // 60/2π
};

#endif // WHEELS_DRIVER_HPP