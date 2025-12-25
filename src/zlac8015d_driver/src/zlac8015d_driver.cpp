#include "zlac8015d_driver.hpp"
#include <cstring>

/* =====================================================
 * Constructor
 * ===================================================== */
ZLAC8015DDriver::ZLAC8015DDriver(CAN& can_interface, uint8_t device_id)
: can_(can_interface), dev_id_(device_id)
{
}

/* =====================================================
 * Low-level SDO sender
 * ===================================================== */
bool ZLAC8015DDriver::send_sdo(const uint8_t data[8])
{
    return can_.send_frame(dev_id_, data, 8);
}

/* =====================================================
 * Set velocity mode
 * ===================================================== */
bool ZLAC8015DDriver::set_velocity_mode()
{
    // 0x6060 = mode of operation, 0x03 = velocity
    const uint8_t data[8] = {
        0x2F, 0x60, 0x60, 0x00,
        0x03, 0x00, 0x00, 0x00
    };
    return send_sdo(data);
}

/* =====================================================
 * Enable driver
 * ===================================================== */
bool ZLAC8015DDriver::enable()
{
    const uint8_t shutdown[8] = {
        0x2B, 0x40, 0x60, 0x00,
        0x06, 0x00, 0x00, 0x00
    };

    const uint8_t switch_on[8] = {
        0x2B, 0x40, 0x60, 0x00,
        0x07, 0x00, 0x00, 0x00
    };

    if (!send_sdo(shutdown))
        return false;

    return send_sdo(switch_on);
}

/* =====================================================
 * Set left & right wheel speed (RPM)
 * ===================================================== */
bool ZLAC8015DDriver::set_sync_left_right_speed(float left_rpm, float right_rpm)
{
    int16_t left  = static_cast<int16_t>(left_rpm);
    int16_t right = static_cast<int16_t>(right_rpm);

    const uint8_t data[8] = {
        0x23, 0xFF, 0x60, 0x03,
        static_cast<uint8_t>(left & 0xFF),
        static_cast<uint8_t>((left >> 8) & 0xFF),
        static_cast<uint8_t>(right & 0xFF),
        static_cast<uint8_t>((right >> 8) & 0xFF)
    };

    return send_sdo(data);
}

/* =====================================================
 * Read wheel speed feedback (RPM)
 * ===================================================== */
bool ZLAC8015DDriver::read_speed_feedback(float& left_rpm, float& right_rpm)
{
    const uint8_t request[8] = {
        0x40, 0x6C, 0x60, 0x03,
        0x00, 0x00, 0x00, 0x00
    };

    uint8_t response[8] = {0};
    uint8_t id = 0;
    uint8_t len = 0;

    if (!send_sdo(request))
        return false;

    if (!can_.receive_frame(id, response, len))
        return false;

    if (len < 8)
        return false;

    int16_t left  = static_cast<int16_t>(response[4] | (response[5] << 8));
    int16_t right = static_cast<int16_t>(response[6] | (response[7] << 8));

    left_rpm  = static_cast<float>(left);
    right_rpm = static_cast<float>(right);

    return true;
}

/* =====================================================
 * Emergency stop
 * ===================================================== */
bool ZLAC8015DDriver::emergency_stop()
{
    const uint8_t data[8] = {
        0x2B, 0x40, 0x60, 0x00,
        0x02, 0x00, 0x00, 0x00
    };
    return send_sdo(data);
}

/* =====================================================
 * Release emergency stop
 * ===================================================== */
bool ZLAC8015DDriver::release_emergency_stop()
{
    const uint8_t data[8] = {
        0x2B, 0x40, 0x60, 0x00,
        0x0F, 0x00, 0x00, 0x00
    };
    return send_sdo(data);
}
