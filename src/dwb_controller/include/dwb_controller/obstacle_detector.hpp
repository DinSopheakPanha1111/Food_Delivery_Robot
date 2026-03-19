#ifndef OBSTACLE_DETECTOR_HPP
#define OBSTACLE_DETECTOR_HPP


#include <stdint.h>
#include <vector>
#include <cmath>


void world_to_cell_conversion
(
  float w_x, 
  float w_y, 
  float o_x, 
  float o_y, 
  float r,
  float *c_x, 
  float *c_y
);

void cell_to_world
(
    float o_x, 
    float o_y, 
    float r, 
    float c_x, 
    float c_y,
    float *w_x, 
    float *w_y
);

void get_R_cells
(
    float v_max, 
    float t_predict, 
    float r, 
    uint32_t *R_cell
);

void bounding_box_and_circle_filter
(
    float c_x, float c_y,
    float r,
    float R_metres,
    uint32_t R_cells,
    float *result, 
    uint32_t *count
);

void collect_lethal_cells
(
    const std::vector<int8_t> &grid,
    uint32_t size_x,
    uint32_t size_y,
    float c_robot_x, float c_robot_y,
    float o_x, float o_y,
    float r,
    uint32_t R_cells,
    float R_metres,
    float *world_out,
    uint32_t *count
);

void cluster_lethal_points
(
    float *world_points,    // flat array [wx0,wy0, wx1,wy1, ...]
    uint32_t count,
    float r,                // resolution — threshold = 2r
    int *labels,            // output: cluster id per point
    int *num_clusters       // output: K
);

void get_obstacle_center_and_radius
(
    float *world_points,
    uint32_t count,
    float r,
    float *w_cx,
    float *w_cy,
    float *radius
);

void calculate_obstacle_cost();

#endif //OBSTACLE_DETECTOR_HPP