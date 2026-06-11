#include "dwb_controller/obstacle_detector.hpp"

void world_to_cell_conversion
(
  float w_x, 
  float w_y, 
  float o_x, 
  float o_y, 
  float r,
  float *c_x, 
  float *c_y
)

{

    *c_x = floorl((w_x - o_x) / r);
    *c_y = floorl((w_y - o_y) / r);

}

void cell_to_world
(
    float o_x, 
    float o_y, 
    float r, 
    float c_x, 
    float c_y,
    float *w_x, 
    float *w_y
)

{

    *w_x = o_x + ((c_x + 0.5f) * r);
    *w_y = o_y + ((c_y + 0.5f) * r);

}

void get_R_cells
(
    float v_max, 
    float t_predict, 
    float r, 
    uint32_t *R_cell
)

{

    *R_cell = floorf(v_max * t_predict) / r;

}

void bounding_box_and_circle_filter
(
    float c_x, float c_y,
    float r,
    float R_metres,
    uint32_t R_cells,
    float *result, 
    uint32_t *count
)

{
    
    float R_sq = R_metres * R_metres;
    *count = 0;

    for (int dx = -(int)R_cells; dx <= (int)R_cells; dx++) {
        for (int dy = -(int)R_cells; dy <= (int)R_cells; dy++) {

            float d_sq = (dx * dx + dy * dy) * r * r;

            if (d_sq <= R_sq) 
            {
                result[(*count) * 2]     = c_x + dx;
                result[(*count) * 2 + 1] = c_y + dy;
                (*count)++;
            }
        }
    }

}

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
)

{

    uint32_t max_cells = (2 * R_cells + 1) * (2 * R_cells + 1);
    *count = 0;

    // step 1: get all cells in bounding box + circle filter
    float    candidate_cells[max_cells * 2];
    uint32_t candidate_count = 0;

    bounding_box_and_circle_filter(c_robot_x, c_robot_y, r, R_metres, R_cells,
                                       candidate_cells, &candidate_count);

    // step 2: keep only lethal cells (value >= 99) and convert to world
    for (uint32_t i = 0; i < candidate_count; i++) {
        int cx = (int)candidate_cells[i * 2];
        int cy = (int)candidate_cells[i * 2 + 1];

        // bounds check
        if (cx < 0 || cy < 0 || cx >= (int)size_x || cy >= (int)size_y) continue;

        if (grid[cy * size_x + cx] == 100) {
            world_out[(*count) * 2]     = o_x + ((cx + 0.5f) * r);  // w_x
            world_out[(*count) * 2 + 1] = o_y + ((cy + 0.5f) * r);  // w_y
            (*count)++;
        }
    }

}

void cluster_lethal_points
(
    float *world_points,    // flat array [wx0,wy0, wx1,wy1, ...]
    uint32_t count,
    float r,                // resolution — threshold = 2r
    int *labels,            // output: cluster id per point
    int *num_clusters       // output: K
)

{

    float threshold_sq = (2.0f * r) * (2.0f * r);

    for (uint32_t i = 0; i < count; i++) labels[i] = -1;
        *num_clusters = 0;

        for (uint32_t i = 0; i < count; i++) {
            if (labels[i] != -1) continue;

            labels[i] = *num_clusters;

            for (uint32_t j = i + 1; j < count; j++) {
                if (labels[j] != -1) continue;

                float dwx = world_points[i * 2]     - world_points[j * 2];
                float dwy = world_points[i * 2 + 1] - world_points[j * 2 + 1];

                if ((dwx * dwx + dwy * dwy) <= threshold_sq) {
                    labels[j] = *num_clusters;
                }
            }
            (*num_clusters)++;
        }
}

void get_obstacle_center_and_radius
(
    float *world_points,
    uint32_t count,
    float r,
    float *w_cx,
    float *w_cy,
    float *radius
)

{

        if (count == 0) return;

        float min_x = world_points[0], max_x = world_points[0];
        float min_y = world_points[1], max_y = world_points[1];

        for (uint32_t i = 1; i < count; i++) {
            float wx = world_points[i * 2];
            float wy = world_points[i * 2 + 1];

            if (wx < min_x) min_x = wx;
            if (wx > max_x) max_x = wx;
            if (wy < min_y) min_y = wy;
            if (wy > max_y) max_y = wy;
        }

        *w_cx = (min_x + max_x) * 0.5f;
        *w_cy = (min_y + max_y) * 0.5f;

        float span_x = (max_x - min_x) + r;
        float span_y = (max_y - min_y) + r;
        *radius = 0.5f * sqrtf(span_x * span_x + span_y * span_y);  

}

void calculate_obstacle_cost()
{

    

}