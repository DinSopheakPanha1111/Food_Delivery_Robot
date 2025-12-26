#ifndef SOCKET_CAN_HPP
#define SOCKET_CAN_HPP

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstdint>
#include <string>

class CAN {
public:
    explicit CAN(const std::string& interface = "can0", int bitrate = 1'000'000);
    ~CAN();

    bool enable(bool configure_link = true);
    bool disable(bool bring_link_down = true);

    bool send_frame(uint16_t id, const uint8_t* data, uint8_t len);
    bool receive_frame(uint16_t &id, uint8_t *data, uint8_t &len);

    bool set_new_bitrate(int new_bitrate);

    bool is_interface_up() const;

private:
    bool run_ip(const std::string& cmd);

    std::string iface_;
    int bitrate_;
    int sock_;
};

#endif // SOCKET_CAN_HPP
