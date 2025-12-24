#include "imu_serial.hpp"
#include "protocol.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <iostream>
#include <cmath>

using namespace ybimu;

static int16_t read_i16(const uint8_t* d)
{
    return static_cast<int16_t>(d[0] | (d[1] << 8));
}

static float read_f32(const uint8_t* d)
{
    float f;
    std::memcpy(&f, d, sizeof(float));
    return f;
}

ImuSerial::ImuSerial(const std::string& port, bool debug)
: debug_(debug), running_(true)
{
    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0)
        throw std::runtime_error("Failed to open serial");

    termios tty{};
    tcgetattr(fd_, &tty);
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_lflag = 0;

    tcsetattr(fd_, TCSANOW, &tty);

    if (debug_)
        std::cout << "YbImu Serial Opened @115200\n";
}

ImuSerial::~ImuSerial()
{
    running_ = false;
    if (rx_thread_.joinable())
        rx_thread_.join();
    close(fd_);
}

void ImuSerial::startReceiveThread()
{
    rx_thread_ = std::thread(&ImuSerial::receiveLoop, this);
}

void ImuSerial::receiveLoop()
{
    uint8_t byte;
    while (running_)
    {
        if (read(fd_, &byte, 1) == 1)
            receiveByte(byte);
    }
}

void ImuSerial::receiveByte(uint8_t data)
{
    switch (rx_state_)
    {
        case 0:
            if (data == HEAD1) rx_state_ = 1;
            break;
        case 1:
            if (data == HEAD2) rx_state_ = 2;
            else rx_state_ = 0;
            break;
        case 2:
            data_len_ = data;
            rx_state_ = (data_len_ <= RX_MAX_LEN) ? 3 : 0;
            break;
        case 3:
            data_func_ = data;
            rx_data_.clear();
            rx_count_ = 4;
            rx_state_ = 4;
            break;
        case 4:
            rx_data_.push_back(data);
            rx_count_++;
            if (rx_count_ >= data_len_ - 1)
                rx_state_ = 5;
            break;
        case 5:
        {
            uint8_t checksum = HEAD1 + HEAD2 + data_len_ + data_func_;
            for (auto b : rx_data_) checksum += b;
            checksum %= 256;

            if (checksum == data)
                parseData(data_func_, rx_data_);

            rx_state_ = 0;
            break;
        }
    }
}

void ImuSerial::parseData(uint8_t type, const std::vector<uint8_t>& d)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (type == FUNC_REPORT_IMU_RAW)
    {
        ax_ = read_i16(&d[0]) * (16.0f / 32767.0f);
        ay_ = read_i16(&d[2]) * (16.0f / 32767.0f);
        az_ = read_i16(&d[4]) * (16.0f / 32767.0f);

        float gyro_ratio = (2000.0f / 32767.0f) * (M_PI / 180.0f);
        gx_ = read_i16(&d[6])  * gyro_ratio;
        gy_ = read_i16(&d[8])  * gyro_ratio;
        gz_ = read_i16(&d[10]) * gyro_ratio;

        float mag_ratio = 800.0f / 32767.0f;
        mx_ = read_i16(&d[12]) * mag_ratio;
        my_ = read_i16(&d[14]) * mag_ratio;
        mz_ = read_i16(&d[16]) * mag_ratio;
    }
    else if (type == FUNC_REPORT_IMU_EULER)
    {
        roll_  = read_f32(&d[0]);
        pitch_ = read_f32(&d[4]);
        yaw_   = read_f32(&d[8]);
    }
    else if (type == FUNC_REPORT_IMU_QUAT)
    {
        q0_ = read_f32(&d[0]);
        q1_ = read_f32(&d[4]);
        q2_ = read_f32(&d[8]);
        q3_ = read_f32(&d[12]);
    }
    else if (type == FUNC_REPORT_BARO)
    {
        height_ = read_f32(&d[0]);
        temperature_ = read_f32(&d[4]);
        pressure_ = read_f32(&d[8]);
        pressure_contrast_ = read_f32(&d[12]);
    }
}

void ImuSerial::sendCmd(const std::vector<uint8_t>& cmd)
{
    write(fd_, cmd.data(), cmd.size());
}

void ImuSerial::requestData(uint8_t func, uint8_t param)
{
    std::vector<uint8_t> cmd = {HEAD1, HEAD2, 0, FUNC_REQUEST_DATA, func, param};
    cmd[2] = cmd.size() + 1;
    uint8_t sum = 0;
    for (auto b : cmd) sum += b;
    cmd.push_back(sum & 0xFF);
    sendCmd(cmd);
}

/* ===== getters ===== */

std::vector<float> ImuSerial::getAccel()
{ return {ax_, ay_, az_}; }

std::vector<float> ImuSerial::getGyro()
{ return {gx_, gy_, gz_}; }

std::vector<float> ImuSerial::getMag()
{ return {mx_, my_, mz_}; }

std::vector<float> ImuSerial::getEuler(bool deg)
{
    if (deg)
        return {roll_ * 57.29578f, pitch_ * 57.29578f, yaw_ * 57.29578f};
    return {roll_, pitch_, yaw_};
}

std::vector<float> ImuSerial::getQuat()
{ return {q0_, q1_, q2_, q3_}; }

std::vector<float> ImuSerial::getBaro()
{ return {height_, temperature_, pressure_, pressure_contrast_}; }

std::string ImuSerial::getVersion()
{
    versionH_ = versionM_ = versionL_ = -1;
    requestData(FUNC_VERSION);
    usleep(20000);
    if (versionH_ >= 0)
        return "V" + std::to_string(versionH_) + "." +
               std::to_string(versionM_) + "." +
               std::to_string(versionL_);
    return "";
}
