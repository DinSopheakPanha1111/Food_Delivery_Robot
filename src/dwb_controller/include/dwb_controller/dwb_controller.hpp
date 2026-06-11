#ifndef DWB_CONTROLLER_HPP
#define DWB_CONTROLLER_HPP

#include <stdint.h>
#include <cmath>
#include <cfloat>

// ─────────────────────────────────────────────
//  Robot state
// ─────────────────────────────────────────────
struct RobotState
{
    float x     = 0.0f;
    float y     = 0.0f;
    float theta = 0.0f;
    float v     = 0.0f;
    float w     = 0.0f;
};

// ─────────────────────────────────────────────
//  Dynamic window
//  v ∈ [max(v_c - a_v*dt, v_min), min(v_c + a_v*dt, v_max)]
//  w ∈ [max(w_c - a_w*dt, -w_max), min(w_c + a_w*dt, w_max)]
// ─────────────────────────────────────────────
void dynamic_window(
    float  v_c,        // current linear  velocity
    float  w_c,        // current angular velocity
    float  a_v,        // linear  acceleration limit
    float  a_w,        // angular acceleration limit
    float  dt,         // time step
    float  v_min,      // min linear velocity (NOT 0)
    float  v_max,      // max linear velocity
    float  w_max,      // max angular velocity (symmetric)
    float *v_min_out,
    float *v_max_out,
    float *w_min_out,
    float *w_max_out
);

// ─────────────────────────────────────────────
//  Goal cost
//  alpha     = atan2(g_y - e_y, g_x - e_x)
//  delta_phi = |atan2(sin(alpha - theta_e), cos(alpha - theta_e))|
//  C_goal    = k_g * delta_phi
// ─────────────────────────────────────────────
void goal_cost(
    float  e_x,        // end-of-arc robot x
    float  e_y,        // end-of-arc robot y
    float  theta_e,    // end-of-arc robot heading
    float  g_x,        // goal x
    float  g_y,        // goal y
    float  k_g,        // goal cost gain
    float *cost_out
);

// ─────────────────────────────────────────────
//  Obstacle cost  (multiple obstacles)
//  d_i   = sqrt((x_i-o_x)^2 + (y_i-o_y)^2) - r_obs - r_robot
//  d_min = min d_i over all arc points i and all obstacles j
//  C_obs = 1e6        if d_min <= 0  (collision)
//        = k_o/d_min  otherwise
// ─────────────────────────────────────────────
void obstacle_cost(
    const float *arc_x,        // arc sample x  [num_points]
    const float *arc_y,        // arc sample y  [num_points]
    uint32_t     num_points,
    const float *obs_x,        // obstacle centres x [num_obstacles]
    const float *obs_y,        // obstacle centres y [num_obstacles]
    const float *obs_rad,      // obstacle radii     [num_obstacles]
    uint32_t     num_obstacles,
    float        r_robot,      // robot radius
    float        k_o,          // obstacle cost gain
    float       *cost_out
);

// ─────────────────────────────────────────────
//  Velocity cost  (only penalised in open space)
//  C_vel = k_v * (v_max - v_e)   if min_clearance > clearance_thresh
//        = 0                      otherwise (near obstacle)
// ─────────────────────────────────────────────
void velocity_cost(
    float  v_max,
    float  v_e,               // end-of-arc linear velocity
    float  min_clearance,     // pre-computed minimum clearance (metres)
    float  clearance_thresh,  // threshold e.g. 1.5 m
    float  k_v,               // velocity cost gain
    float *cost_out
);

// ─────────────────────────────────────────────
//  Euclidean distance
//  d(A,B) = sqrt((x_A-x_B)^2 + (y_A-y_B)^2)
// ─────────────────────────────────────────────
void euclidean_distance(
    float  x_a, float y_a,
    float  x_b, float y_b,
    float *dist_out
);

// ─────────────────────────────────────────────
//  DWA rollout — (v*, w*) = argmin C_total(v,w)
//  C_total = C_goal + C_obs + C_vel
// ─────────────────────────────────────────────
void rollout_best_control(
    const RobotState &state,
    float  goal_x,
    float  goal_y,
    const float *obs_x,
    const float *obs_y,
    const float *obs_rad,
    uint32_t     num_obstacles,
    float  k_g,
    float  k_o,
    float  k_v,
    float  v_min,
    float  v_max,
    float  w_max,
    float  a_v,
    float  a_w,
    float  dt,
    float  window_time,
    float  vel_res,
    float  ang_res,
    float  r_robot,
    float  clearance_thresh,
    float *best_v_out,
    float *best_w_out
);

#endif // DWB_CONTROLLER_HPP