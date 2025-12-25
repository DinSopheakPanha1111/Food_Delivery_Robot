#ifndef KINEMATIC_HPP
#define KINEMATIC_HPP

#include <cstdint>

class KINEMATIC {
public:

//***************************************************************************** */
/**Function float get_left_wheel_rpm(float v_bx, float omega_bz, float w, float wheel_radius)
 * Input : v_bx : Forward velocity of robot (m/s)
 *         omega_bz : Rotational velocity of robot (rad/s)
 *         w : The distance between the two wheels (m)
 *         wheel_radius : The wheel radius(m)
 * Output :  speed of the left wheel in (rpm)
 * BY : DIN Sopheak Panha
 * gmail: dinsopheakpanha24@gmail.com
 //**************************************************************************** */
float get_left_wheel_rpm(float v_bx, float omega_bz, float w, float wheel_radius);
//***************************************************************************** */
/**Function float get_right_wheel_rpm(float v_bx, float omega_bz, float w, float wheel_radius)
 * Input : v_bx : Forward velocity of robot (m/s)
 *         omega_bz : Rotational velocity of robot (rad/s)
 *         w : The distance between the two wheels (m)
 *         wheel_radius : The wheel radius(m)
 * Output :  speed of the right wheel in (rpm)
 * BY : DIN Sopheak Panha
 * gmail: dinsopheakpanha24@gmail.com
 //**************************************************************************** */
float get_right_wheel_rpm(float v_bx, float omega_bz, float w, float wheel_radius);
//***************************************************************************** */
/**Function float get_forward_velocity(float v_right, float v_left)
 * Input : v_right : Right wheel velocity that feedback from the actual motor in (rpm)
 *         v_left : Left wheel velocity that feedback from the actual motor in (rpm)
 * Output :  Forward velocity of the robot in (m/s)
 * BY : DIN Sopheak Panha
 * gmail: dinsopheakpanha24@gmail.com
 //**************************************************************************** */
float get_forward_velocity(float v_right, float v_left);
//***************************************************************************** */
/**Function float get_rotational_velocity(float v_right, float v_left, float w)
 * Input : v_right : Right wheel velocity that feedback from the actual motor in (rpm)
 *         v_left : Left wheel velocity that feedback from the actual motor in (rpm)
 *         w : The distance between the two wheels (m)
 * Output :  Rotational velocity of the robot in (rad/s)
 * BY : DIN Sopheak Panha
 * gmail: dinsopheakpanha24@gmail.com
 //**************************************************************************** */
float get_rotational_velocity(float v_right, float v_left, float w); //FK, Get rotational velocity, v_right and v_left should be from the motor 

private:
};

#endif // KINEMATIC_HPP
