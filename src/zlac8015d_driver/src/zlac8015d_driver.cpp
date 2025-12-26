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
 * TX COB-ID = 0x600 + node_id  (node_id=0x01 -> 0x601)
 * ===================================================== */
bool ZLAC8015DDriver::send_sdo(const uint8_t data[8])
{
    const uint16_t tx_id = static_cast<uint16_t>(0x600u + dev_id_);
    return can_.send_frame(tx_id, data, 8);
}

/* =====================================================
 * Low-level SDO receiver
 * RX COB-ID = 0x580 + node_id  (node_id=0x01 -> 0x581)
 * ===================================================== */
bool ZLAC8015DDriver::receive_sdo(uint8_t response[8])
{
    uint16_t rx_id = 0;
    uint8_t len = 0;

    if (!can_.receive_frame(rx_id, response, len))
        return false;

    const uint16_t expected_rx_id = static_cast<uint16_t>(0x580u + dev_id_);
    if (rx_id != expected_rx_id)
        return false;

    return (len == 8);
}

/* =====================================================
 * Set velocity mode
 * 0x6060 = mode of operation, 0x03 = velocity
 * ===================================================== */
bool ZLAC8015DDriver::set_velocity_mode()
{
    const uint8_t data[8] = {
        0x2F, 0x60, 0x60, 0x00,  // write 8-bit to 0x6060:00
        0x03, 0x00, 0x00, 0x00   // velocity mode
    };
    return send_sdo(data);
}

/* =====================================================
 * Enable driver (CiA-402 Controlword 0x6040)
 * Shutdown (0x06) -> Switch on (0x07) -> Enable op (0x0F)
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

    const uint8_t enable_op[8] = {
        0x2B, 0x40, 0x60, 0x00,
        0x0F, 0x00, 0x00, 0x00
    };

    if (!send_sdo(shutdown))  return false;
    if (!send_sdo(switch_on)) return false;
    return send_sdo(enable_op);
}

/* =====================================================
 * Set left & right wheel speed (RPM)
 * Object: 0x60FF:03 (as per your usage)
 * NOTE: This packs int16 left/right into bytes 4..7
 * ===================================================== */
bool ZLAC8015DDriver::set_sync_left_right_speed(float left_rpm, float right_rpm)
{
    int16_t left  = static_cast<int16_t>(left_rpm);
    int16_t right = static_cast<int16_t>(right_rpm);

    const uint8_t data[8] = {
        0x23, 0xFF, 0x60, 0x03,   // write 32-bit to 0x60FF:03 (you pack 2x int16)
        static_cast<uint8_t>(left & 0xFF),
        static_cast<uint8_t>((left >> 8) & 0xFF),
        static_cast<uint8_t>(right & 0xFF),
        static_cast<uint8_t>((right >> 8) & 0xFF)
    };

    return send_sdo(data);
}

/* =====================================================
 * Read wheel speed feedback (RPM)
 * Object: 0x606C:03
 * ===================================================== */
bool ZLAC8015DDriver::read_speed_feedback(float& left_rpm, float& right_rpm)
{
    const uint8_t request[8] = {
        0x40, 0x6C, 0x60, 0x03,   // read 0x606C:03
        0x00, 0x00, 0x00, 0x00
    };

    uint8_t response[8] = {0};

    if (!send_sdo(request))
        return false;

    if (!receive_sdo(response))
        return false;

    // response[4..7] contain the data (you interpret as 2x int16)
    int16_t left  = static_cast<int16_t>(response[4] | (response[5] << 8));
    int16_t right = static_cast<int16_t>(response[6] | (response[7] << 8));

    left_rpm  = static_cast<float>(left);
    right_rpm = static_cast<float>(right);

    return true;
}

/* =====================================================
 * Emergency stop (controlword 0x6040 = 0x02)
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
 * Release emergency stop (back to Enable operation)
 * ===================================================== */
bool ZLAC8015DDriver::release_emergency_stop()
{
    const uint8_t data[8] = {
        0x2B, 0x40, 0x60, 0x00,
        0x0F, 0x00, 0x00, 0x00
    };
    return send_sdo(data);
}
