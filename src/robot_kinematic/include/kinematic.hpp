#ifndef KINEMATIC_HPP
#define KINEMATIC_HPP

#include <cstdint>

class KINEMATIC {
public:

void forward_kinematic(float w_right_rpm, float w_left_rpm, float r, float d, float *v, float *omega);
void inverse_kinematic(float v_cmd, float omega_cmd, float r, float d,
                                  float *cnd_w_right, float *cmd_w_left); 

private:
};

#endif // KINEMATIC_HPP
