#ifndef ZLAC8015D_DRIVER_HPP
#define ZLAC8015D_DRIVER_HPP

#include "can.hpp"
#include <cstdint>

class ZLAC8015DDriver {
public:
    explicit ZLAC8015DDriver(CAN& can_interface, uint8_t device_id);

    /* ================= Mode & State ================= */
    bool set_velocity_mode();
    bool enable();
    bool emergency_stop();
    bool release_emergency_stop();

    /* ================= Motion ================= */
    bool set_sync_left_right_speed(float left_rpm, float right_rpm);

    /* ================= Feedback ================= */
    bool read_speed_feedback(float& left_rpm, float& right_rpm);

private:
    CAN& can_;
    uint8_t dev_id_;

    /* ================= SDO Helpers ================= */
    bool send_sdo(const uint8_t data[8]);      // TX → 0x600 + node_id
    bool receive_sdo(uint8_t response[8]);     // RX ← 0x580 + node_id
};

#endif // ZLAC8015D_DRIVER_HPP
