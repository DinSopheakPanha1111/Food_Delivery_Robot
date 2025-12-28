// # ===========================
// #  By : Din Sopheak Panha
// # ===========================

#include "can.hpp"
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <sys/socket.h>
#include <errno.h>

CAN::CAN(const std::string& interface, int bitrate)
: iface_(interface), bitrate_(bitrate), sock_(-1) {}

CAN::~CAN()
{
    disable(true);
}

bool CAN::run_ip(const std::string& ip_cmd)
{
    const char* IP = "/sbin/ip";
    std::string base = std::string((access(IP, X_OK) == 0) ? IP : "ip") + " " + ip_cmd;
    std::string cmd  = (geteuid() == 0) ? base : "sudo " + base;

    int rc = std::system(cmd.c_str());
    if (rc == -1) return false;

#ifdef WEXITSTATUS
    return (WEXITSTATUS(rc) == 0);
#else
    return (rc == 0);
#endif
}

// ======================================================
bool CAN::is_interface_up() const
{
    FILE* f = popen(("ip link show " + iface_).c_str(), "r");
    if (!f) return false;

    char buf[256];
    std::string out;
    while (fgets(buf, sizeof(buf), f))
        out += buf;

    pclose(f);

    return out.find("state UP") != std::string::npos;
}

// ======================================================
bool CAN::enable(bool configure_link)
{
    if (sock_ != -1)
        return true;

    if (configure_link && !is_interface_up())
    {
        (void)run_ip("link set " + iface_ + " down");

        if (!run_ip("link set " + iface_ +
                    " type can bitrate " + std::to_string(bitrate_)))
            return false;

        if (!run_ip("link set " + iface_ + " up"))
            return false;
    }

    sock_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) {
        perror("socket");
        return false;
    }

    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, iface_.c_str(), IFNAMSIZ - 1);

    if (::ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl SIOCGIFINDEX");
        ::close(sock_);
        sock_ = -1;
        return false;
    }

    struct sockaddr_can addr {};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (::bind(sock_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        ::close(sock_);
        sock_ = -1;
        return false;
    }

    return true;
}

// ======================================================
bool CAN::disable(bool bring_link_down)
{
    if (sock_ != -1) {
        ::close(sock_);
        sock_ = -1;
    }

    if (bring_link_down)
        (void)run_ip("link set " + iface_ + " down");

    return true;
}

// ======================================================
// 
bool CAN::send_frame(uint16_t id, const uint8_t* data, uint8_t len)
{
    if (sock_ < 0 || len > 8)
        return false;

    struct can_frame frame {};
    frame.can_id  = id & CAN_SFF_MASK;
    frame.can_dlc = len;

    if (len)
        std::memcpy(frame.data, data, len);

    return (::write(sock_, &frame, sizeof(frame)) ==
            static_cast<ssize_t>(sizeof(frame)));
}

// ======================================================
// receive full CAN ID
bool CAN::receive_frame(uint16_t &id, uint8_t *data, uint8_t &len)
{
    if (sock_ < 0)
        return false;

    struct can_frame frame {};

    struct timeval tv {2, 0};
    setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    if (::read(sock_, &frame, sizeof(frame)) != sizeof(frame))
        return false;

    id  = frame.can_id & CAN_SFF_MASK;
    len = frame.can_dlc;

    if (len)
        std::memcpy(data, frame.data, len);

    return true;
}

// ======================================================
bool CAN::set_new_bitrate(int new_bitrate)
{
    std::cout << "[CAN] Changing bitrate from "
              << bitrate_ << " to " << new_bitrate << "...\n";

    disable(true);

    if (!run_ip("link set " + iface_ + " down") ||
        !run_ip("link set " + iface_ +
                " type can bitrate " + std::to_string(new_bitrate)) ||
        !run_ip("link set " + iface_ + " up"))
    {
        std::cerr << "[CAN] Failed to reconfigure bitrate!\n";
        return false;
    }

    bitrate_ = new_bitrate;

    if (!enable(false)) {
        std::cerr << "[CAN] Failed to reopen socket after bitrate change!\n";
        return false;
    }

    std::cout << "[CAN] Bitrate successfully changed\n";
    return true;
}
