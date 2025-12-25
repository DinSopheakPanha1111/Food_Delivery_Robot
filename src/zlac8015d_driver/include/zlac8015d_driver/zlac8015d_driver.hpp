#ifndef ZLAC8015D_DRIVER_HPP
#define ZLAC8015D_DRIVER_HPP

#include "can.hpp"
#include <cstdint>

class ZLAC8015DDriver {
public:
    explicit ZLAC8015DDriver(CAN& can_interface, uint8_t device_id);

    // Mode & state
    bool set_velocity_mode();
    bool enable();
    bool emergency_stop();
    bool release_emergency_stop();

    // Motion
    bool set_sync_left_right_speed(float left_rpm, float right_rpm);

    // Feedback
    bool read_speed_feedback(float& left_rpm, float& right_rpm);

private:
    CAN& can_;
    uint8_t dev_id_;

    bool send_sdo(const uint8_t data[8]);
};

#endif // ZLAC8015D_DRIVER_HPP
