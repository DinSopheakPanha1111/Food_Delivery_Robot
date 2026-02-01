#include "food_del_robot_hardware/wheels_driver.hpp"
#include <cmath>

/* =====================================================
 * Constructor
 * ===================================================== */
WheelsDriver::WheelsDriver(const std::string& can_interface, int bitrate, uint8_t device_id)
    : can_interface_name_(can_interface)
    , can_bitrate_(bitrate)
    , device_id_(device_id)
{
    // Note: CAN and driver objects are created in init()
}

/* =====================================================
 * Destructor
 * ===================================================== */
WheelsDriver::~WheelsDriver()
{
    // Smart pointers will automatically clean up
}

/* =====================================================
 * Initialize - Create CAN object and driver object
 * ===================================================== */
bool WheelsDriver::init()
{
    // ============================================
    // CREATE CAN OBJECT
    // ============================================
    can_interface_ = std::make_unique<CAN>(can_interface_name_, can_bitrate_);
    
    // ============================================
    // CREATE ZLAC8015D DRIVER OBJECT USING THE CAN OBJECT
    // ============================================
    zlac_driver_ = std::make_unique<ZLAC8015DDriver>(*can_interface_, device_id_);
    
    // Enable CAN interface
    if (!can_interface_->enable(false)) {
        return false;
    }
    
    return true;
}

/* =====================================================
 * Mode & State Functions (simply forward to ZLAC driver)
 * ===================================================== */
bool WheelsDriver::set_velocity_mode()
{
    if (!zlac_driver_) return false;
    return zlac_driver_->set_velocity_mode();
}

bool WheelsDriver::enable()
{
    if (!zlac_driver_) return false;
    return zlac_driver_->enable();
}

bool WheelsDriver::emergency_stop()
{
    if (!zlac_driver_) return false;
    return zlac_driver_->emergency_stop();
}

bool WheelsDriver::status_emergency_stop()
{
    if (!zlac_driver_) return false;
    return zlac_driver_->status_emergency_stop();
}

bool WheelsDriver::release_emergency_stop()
{
    if (!zlac_driver_) return false;
    return zlac_driver_->release_emergency_stop();
}

bool WheelsDriver::status_release_emergency_stop()
{
    if (!zlac_driver_) return false;
    return zlac_driver_->status_release_emergency_stop();
}

bool WheelsDriver::shutdown()
{
    if (!zlac_driver_) return false;
    return zlac_driver_->shutdown();
}

/* =====================================================
 * Set left & right wheel speed (input in rad/s)
 * Converts rad/s to RPM for the ZLAC driver
 * ===================================================== */
bool WheelsDriver::set_sync_left_right_speed(float left_rad, float right_rad)
{
    if (!zlac_driver_) return false;
    
    // Convert rad/s to RPM
    float left_rpm = left_rad * RAD_PER_SEC_TO_RPM;
    float right_rpm = right_rad * RAD_PER_SEC_TO_RPM;
    
    return zlac_driver_->set_sync_left_right_speed(left_rpm, right_rpm);
}

/* =====================================================
 * Read wheel speed feedback (output in rad/s)
 * Gets RPM from ZLAC driver and converts to rad/s
 * ===================================================== */
bool WheelsDriver::get_speed_rad_per_sec(float& left_rad, float& right_rad)
{
    if (!zlac_driver_) return false;
    
    float left_rpm = 0.0f;
    float right_rpm = 0.0f;
    
    // Get RPM from ZLAC driver
    if (!zlac_driver_->read_speed_feedback(left_rpm, right_rpm)) {
        return false;
    }
    
    // Convert RPM to rad/s
    left_rad = left_rpm * RPM_TO_RAD_PER_SEC;
    right_rad = right_rpm * RPM_TO_RAD_PER_SEC;
    
    return true;
}