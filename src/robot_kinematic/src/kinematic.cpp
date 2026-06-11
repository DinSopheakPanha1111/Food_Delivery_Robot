#include "kinematic.hpp"
#include <cmath>   // for M_PI

void KINEMATIC::forward_kinematic(float w_right_rpm, float w_left_rpm, float r, float d, float *v, float *omega)
{

    float w_right = w_right_rpm * 0.1f * (M_PI / 30.0f);  // rpm -> rad/s
    float w_left  = w_left_rpm *0.1f  * (M_PI / 30.0f);  // rpm -> rad/s

    *v = r * ((w_right + w_left) / 2.0f);
    *omega = r * ((w_right - w_left) / d);

}
void KINEMATIC::inverse_kinematic(float v_cmd, float omega_cmd, float r, float d,
                                  float *cnd_w_right, float *cmd_w_left)
{
    // wheel angular speed in rad/s
    float w_right_rad = (v_cmd / r) + ((d * omega_cmd) / (2.0f * r));
    float w_left_rad  = (v_cmd / r) - ((d * omega_cmd) / (2.0f * r));
    // convert rad/s -> rpm
    float RAD_TO_RPM = 30.0f / M_PI;

    *cnd_w_right = w_right_rad * RAD_TO_RPM;
    *cmd_w_left = w_left_rad * RAD_TO_RPM;
}