#include "dwb_controller/dwb_controller.hpp"

// ─────────────────────────────────────────────
//  Dynamic window
// ─────────────────────────────────────────────
void dynamic_window(
    float  v_c,
    float  w_c,
    float  a_v,
    float  a_w,
    float  dt,
    float  v_min,
    float  v_max,
    float  w_max,
    float *v_min_out,
    float *v_max_out,
    float *w_min_out,
    float *w_max_out)
{
    *v_min_out = fmaxf(v_c - a_v * dt,  v_min);   // clamp to v_min, NOT 0
    *v_max_out = fminf(v_c + a_v * dt,  v_max);
    *w_min_out = fmaxf(w_c - a_w * dt, -w_max);
    *w_max_out = fminf(w_c + a_w * dt,  w_max);
}

// ─────────────────────────────────────────────
//  Goal cost
// ─────────────────────────────────────────────
void goal_cost(
    float  e_x,
    float  e_y,
    float  theta_e,
    float  g_x,
    float  g_y,
    float  k_g,
    float *cost_out)
{
    float alpha     = atan2f(g_y - e_y, g_x - e_x);
    float diff      = alpha - theta_e;
    float delta_phi = fabsf(atan2f(sinf(diff), cosf(diff)));  // normalised to [-pi, pi]
    *cost_out       = k_g * delta_phi;
}

// ─────────────────────────────────────────────
//  Obstacle cost
// ─────────────────────────────────────────────
void obstacle_cost(
    const float *arc_x,
    const float *arc_y,
    uint32_t     num_points,
    const float *obs_x,
    const float *obs_y,
    const float *obs_rad,
    uint32_t     num_obstacles,
    float        r_robot,
    float        k_o,
    float       *cost_out)
{
    if (num_points == 0 || num_obstacles == 0)
    {
        *cost_out = 0.0f;
        return;
    }

    float d_min = FLT_MAX;

    for (uint32_t i = 0; i < num_points; i++)
    {
        for (uint32_t j = 0; j < num_obstacles; j++)
        {
            float dx = arc_x[i] - obs_x[j];
            float dy = arc_y[i] - obs_y[j];
            float d  = sqrtf(dx*dx + dy*dy) - obs_rad[j] - r_robot;
            if (d < d_min) d_min = d;
        }
    }

    if (d_min <= 0.0f)
        *cost_out = 1e6f;           // collision — arc rejected
    else
        *cost_out = k_o / d_min;
}

// ─────────────────────────────────────────────
//  Velocity cost
// ─────────────────────────────────────────────
void velocity_cost(
    float  v_max,
    float  v_e,
    float  min_clearance,
    float  clearance_thresh,
    float  k_v,
    float *cost_out)
{
    if (min_clearance > clearance_thresh)
        *cost_out = k_v * (v_max - v_e);   // open space: penalise slow speed
    else
        *cost_out = 0.0f;                  // near obstacle: don't penalise speed
}

// ─────────────────────────────────────────────
//  Euclidean distance
// ─────────────────────────────────────────────
void euclidean_distance(
    float  x_a, float y_a,
    float  x_b, float y_b,
    float *dist_out)
{
    float dx  = x_a - x_b;
    float dy  = y_a - y_b;
    *dist_out = sqrtf(dx*dx + dy*dy);
}

// ─────────────────────────────────────────────
//  Internal helpers
// ─────────────────────────────────────────────

// forward-simulate one arc and fill arc_x / arc_y sample arrays
static void simulate_arc(
    const RobotState &state,
    float  v, float w,
    float  dt, float window_time,
    float *arc_x, float *arc_y,
    uint32_t *num_pts_out,
    uint32_t  max_pts,
    RobotState *end_state_out)
{
    RobotState s = state;
    uint32_t   n = 0;
    float      t = 0.0f;

    while (t <= window_time && n < max_pts)
    {
        arc_x[n] = s.x;
        arc_y[n] = s.y;
        n++;

        float theta_new = s.theta + w * dt;
        s.x    += v * cosf(theta_new) * dt;
        s.y    += v * sinf(theta_new) * dt;
        s.theta = theta_new;
        s.v     = v;
        s.w     = w;
        t      += dt;
    }

    *num_pts_out   = n;
    *end_state_out = s;
}

// compute minimum clearance over entire arc (for velocity_cost)
static float arc_min_clearance(
    const float *arc_x, const float *arc_y, uint32_t num_pts,
    const float *obs_x, const float *obs_y, const float *obs_rad,
    uint32_t num_obstacles, float r_robot)
{
    float d_min = FLT_MAX;
    for (uint32_t i = 0; i < num_pts; i++)
    {
        for (uint32_t j = 0; j < num_obstacles; j++)
        {
            float dx = arc_x[i] - obs_x[j];
            float dy = arc_y[i] - obs_y[j];
            float d  = sqrtf(dx*dx + dy*dy) - obs_rad[j] - r_robot;
            if (d < d_min) d_min = d;
        }
    }
    return d_min;
}

// ─────────────────────────────────────────────
//  DWA rollout
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
    float *best_w_out)
{
    // max arc steps = ceil(window_time / dt) + 1
    constexpr uint32_t MAX_ARC_PTS = 32;

    float dw_v_min, dw_v_max, dw_w_min, dw_w_max;
    dynamic_window(state.v, state.w,
                   a_v, a_w, dt,
                   v_min, v_max, w_max,
                   &dw_v_min, &dw_v_max,
                   &dw_w_min, &dw_w_max);

    float    min_cost = FLT_MAX;
    float    arc_x[MAX_ARC_PTS];
    float    arc_y[MAX_ARC_PTS];

    // initialise output to safe fallback
    *best_v_out = v_min;
    *best_w_out = 0.0f;

    for (float v = dw_v_min; v <= dw_v_max + 1e-6f; v += vel_res)
    {
        for (float w = dw_w_min; w <= dw_w_max + 1e-6f; w += ang_res)
        {
            // 1. simulate arc
            uint32_t   num_pts;
            RobotState end;
            simulate_arc(state, v, w, dt, window_time,
                         arc_x, arc_y, &num_pts, MAX_ARC_PTS, &end);

            // 2. goal cost
            float c_goal;
            goal_cost(end.x, end.y, end.theta,
                      goal_x, goal_y, k_g, &c_goal);

            // 3. obstacle cost
            float c_obs;
            obstacle_cost(arc_x, arc_y, num_pts,
                          obs_x, obs_y, obs_rad, num_obstacles,
                          r_robot, k_o, &c_obs);

            // skip immediately on collision
            if (c_obs >= 1e6f) continue;

            // 4. velocity cost (needs clearance)
            float clearance = arc_min_clearance(arc_x, arc_y, num_pts,
                                                obs_x, obs_y, obs_rad,
                                                num_obstacles, r_robot);
            float c_vel;
            velocity_cost(v_max, end.v, clearance,
                          clearance_thresh, k_v, &c_vel);

            // 5. total cost  C_total = C_goal + C_obs + C_vel
            float c_total = c_goal + c_obs + c_vel;

            if (c_total < min_cost)
            {
                min_cost    = c_total;
                *best_v_out = v;
                *best_w_out = w;
            }
        }
    }
}