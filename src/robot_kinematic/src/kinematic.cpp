#include "kinematic.hpp"
#include <cmath>   // for M_PI

// ================= LEFT WHEEL =================
float KINEMATIC::get_left_wheel_rpm(float v_bx,
                                    float omega_bz,
                                    float w,
                                    float wheel_radius)
{
    // Linear velocity of left wheel (m/s)
    float v_left = v_bx - (omega_bz * w) / 2.0f;

    // Angular speed (rad/s)
    float omega_left_rad = v_left / wheel_radius;

    // Convert to RPM
    float omega_left_rpm = omega_left_rad * 60.0f / (2.0f * M_PI);

    return omega_left_rpm;
}

// ================= RIGHT WHEEL =================
float KINEMATIC::get_right_wheel_rpm(float v_bx,
                                     float omega_bz,
                                     float w,
                                     float wheel_radius)
{
    // Linear velocity of right wheel (m/s)
    float v_right = v_bx + (omega_bz * w) / 2.0f;

    // Angular speed (rad/s)
    float omega_right_rad = v_right / wheel_radius;

    // Convert to RPM
    float omega_right_rpm = omega_right_rad * 60.0f / (2.0f * M_PI);

    return omega_right_rpm;
}

// ================= FORWARD VELOCITY =================
float KINEMATIC::get_forward_velocity(float v_right, float v_left)
{
    // Average linear velocity (m/s)
    float v_bx = (v_right + v_left) / 2.0f;
    return v_bx;
}

// ================= ROTATIONAL VELOCITY =================
float KINEMATIC::get_rotational_velocity(float v_right,
                                         float v_left,
                                         float w)
{
    // Angular velocity around z (rad/s)
    float omega_bz = (v_right - v_left) / w;
    return omega_bz;
}
