#pragma once

#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <cstdint>

namespace ybimu
{

class ImuSerial
{
public:
    explicit ImuSerial(const std::string& port, bool debug=false);
    ~ImuSerial();

    void startReceiveThread();

    // getters
    std::vector<float> getAccel();
    std::vector<float> getGyro();
    std::vector<float> getMag();
    std::vector<float> getEuler(bool to_degree=true);
    std::vector<float> getQuat();
    std::vector<float> getBaro();

    std::string getVersion();

    void setReportRate(uint8_t rate);
    void setAlgoType(uint8_t algo);

private:
    void receiveLoop();
    void receiveByte(uint8_t data);
    void parseData(uint8_t type, const std::vector<uint8_t>& payload);
    void sendCmd(const std::vector<uint8_t>& cmd);
    void requestData(uint8_t func, uint8_t param=0);

    int fd_;
    bool debug_;
    std::atomic<bool> running_;
    std::thread rx_thread_;
    std::mutex data_mutex_;

    // rx state machine
    uint8_t rx_state_{0};
    uint8_t data_len_{0};
    uint8_t data_func_{0};
    uint8_t rx_count_{0};
    std::vector<uint8_t> rx_data_;

    // data
    float ax_, ay_, az_;
    float gx_, gy_, gz_;
    float mx_, my_, mz_;
    float roll_, pitch_, yaw_;
    float q0_, q1_, q2_, q3_;
    float height_, temperature_, pressure_, pressure_contrast_;

    int versionH_, versionM_, versionL_;
};

}
