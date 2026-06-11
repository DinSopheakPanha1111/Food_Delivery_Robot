#pragma once
#include <cstdint>

namespace ybimu
{
    constexpr uint8_t HEAD1 = 0x7E;
    constexpr uint8_t HEAD2 = 0x23;

    constexpr uint8_t FUNC_VERSION          = 0x01;
    constexpr uint8_t FUNC_REPORT_IMU_RAW   = 0x04;
    constexpr uint8_t FUNC_REPORT_IMU_QUAT  = 0x16;
    constexpr uint8_t FUNC_REPORT_IMU_EULER = 0x26;
    constexpr uint8_t FUNC_REPORT_BARO      = 0x32;

    constexpr uint8_t FUNC_REPORT_RATE      = 0x60;
    constexpr uint8_t FUNC_ALGO_TYPE        = 0x61;

    constexpr uint8_t FUNC_CALIB_IMU        = 0x70;
    constexpr uint8_t FUNC_CALIB_MAG        = 0x71;
    constexpr uint8_t FUNC_CALIB_BARO       = 0x72;
    constexpr uint8_t FUNC_CALIB_TEMP       = 0x73;

    constexpr uint8_t FUNC_REQUEST_DATA     = 0x80;
    constexpr uint8_t FUNC_RETURN_STATE     = 0x81;

    constexpr uint8_t FUNC_RESET_FLASH      = 0xA0;

    constexpr uint8_t RX_MAX_LEN             = 40;
}
