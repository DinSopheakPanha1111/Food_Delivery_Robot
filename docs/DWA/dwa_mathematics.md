From 

        /local_costmap/costmap  
        and 
        /local_costmap/costmap_updates

We have :

nav_msgs/msg/OccupancyGrid (Publish only once to get full local costmap): 

        # This represents a 2-D grid map
        std_msgs/Header header
            builtin_interfaces/Time stamp
                int32 sec
                uint32 nanosec
            string frame_id

        # MetaData for the map
        MapMetaData info
            builtin_interfaces/Time map_load_time
                int32 sec
                uint32 nanosec
            float32 resolution
            uint32 width
            uint32 height
            geometry_msgs/Pose origin
                Point position
                    float64 x
                    float64 y
                    float64 z
                Quaternion orientation
                    float64 x 0
                    float64 y 0
                    float64 z 0
                    float64 w 1

map_msgs/msg/OccupancyGridUpdate (Update frequently)

        std_msgs/Header header
            builtin_interfaces/Time stamp
                int32 sec
                uint32 nanosec
            string frame_id
        int32 x
        int32 y
        uint32 width
        uint32 height
        int8[] data

# DWB Local Costmap — Obstacle Detection Mathematics

## 0. Annotations

| Symbol | Description |
|---|---|
| $(w_x, w_y)$ | Robot world position (metres) |
| $(o_x, o_y)$ | Costmap origin — bottom-left corner (metres) |
| $r$ | Resolution (metres/cell) |
| $W$ | Costmap width in cells |
| $H$ | Costmap height in cells |
| $(c_x, c_y)$ | Cell index (column, row) — no units |
| $(\delta_x, \delta_y)$ | Cell offset from the robot cell |
| $R_{metres}$ | Search radius (metres) |
| $R_{cells}$ | Search radius (cells) |
| $v_{max}$ | Maximum robot velocity (m/s) |
| $t_{predict}$ | Prediction time (s) |
| $\mathcal{G}$ | Full costmap grid |
| $\Delta\mathcal{G}$ | Costmap update patch |
| $\mathcal{C}$ | Set of lethal cells |
| $\mathcal{P}$ | Set of lethal points in world frame |
| $N_k$ | Number of points in cluster $k$ |
| $(\bar{w}_x^k, \bar{w}_y^k)$ | Centre of obstacle $k$ |
| $\rho_k$ | Radius of obstacle $k$ |

---

## 1. Initialize and Patch the Grid

**From `/local_costmap/costmap` (full map, received once at start):**

$$\mathcal{G}[c_y][c_x] = \text{msg.data}[c_y \cdot W + c_x]$$

**From `/local_costmap/costmap_updates` (patch, received every cycle):**

$$\mathcal{G}[y_0 + \delta_y][x_0 + \delta_x] = \Delta\mathcal{G}[\delta_y \cdot \Delta W + \delta_x]$$

$$\forall\ \delta_x \in [0,\ \Delta W), \quad \delta_y \in [0,\ \Delta H)$$

---

## 2. World to Cell Conversion

From `/odom`: robot position $(w_x,\ w_y)$ in world frame (metres).

$$c_x = \left\lfloor \frac{w_x - o_x}{r} \right\rfloor, \qquad c_y = \left\lfloor \frac{w_y - o_y}{r} \right\rfloor$$

Cell to flat 1D array index:

$$\text{index} = c_y \cdot W + c_x$$

---

## 3. Search Radius in Cells

$$R_{cells} = \left\lceil \frac{R_{metres}}{r} \right\rceil$$

Two ways to define $R_{metres}$:

$$R_{metres} = \frac{\text{costmap\_width} \cdot r}{2} \qquad \text{(use full local costmap)}$$

$$\text{or}$$

$$R_{metres} = v_{max} \times t_{predict} \qquad \text{(how far robot can travel)}$$

---

## 4. Bounding Box and Circle Filter

Scan all cells $(\delta_x,\ \delta_y)$ in the bounding box:

$$\delta_x \in [-R_{cells},\ R_{cells}], \qquad \delta_y \in [-R_{cells},\ R_{cells}]$$

Total cells in box: $(2R_{cells} + 1)^2$

Keep only cells inside the **circle**:

$$d = \sqrt{\delta_x^2 + \delta_y^2} \cdot r \leq R_{metres}$$

---

## 5. Collect Lethal Cells

$$\mathcal{C} = \left\{(c_x^{robot} + \delta_x,\ c_y^{robot} + \delta_y)\ \Big|\ d \leq R_{metres}\ \wedge\ \mathcal{G}[c_y][c_x] = 100\right\}$$

Convert each lethal cell to world coordinates (cell centre):

$$w_x^i = o_x + \left(c_x^i + \frac{1}{2}\right) \cdot r, \qquad w_y^i = o_y + \left(c_y^i + \frac{1}{2}\right) \cdot r$$

$$\mathcal{P} = \{(w_x^i,\ w_y^i)\ |\ \forall\ (c_x^i,\ c_y^i) \in \mathcal{C}\}$$

> **Note:** $+\frac{1}{2}$ shifts from the corner of the cell to its centre

---

## 6. Cluster Lethal Points

Two points $i, j$ belong to the **same obstacle** if:

$$d_{ij} = \sqrt{(w_x^i - w_x^j)^2 + (w_y^i - w_y^j)^2} \leq 2r$$

Result: $K$ clusters $\mathcal{C}_1,\ \mathcal{C}_2,\ \ldots,\ \mathcal{C}_K$

---

## 7. Obstacle Centre and Radius

For each cluster $\mathcal{C}_k$ of size $N_k$:

**Centre (centroid):**

$$\bar{w}_x^k = \frac{1}{N_k}\sum_{i \in \mathcal{C}_k} w_x^i, \qquad \bar{w}_y^k = \frac{1}{N_k}\sum_{i \in \mathcal{C}_k} w_y^i$$

**Radius** (max distance from centroid to any point + safety margin):

$$\rho_k = \max_{i \in \mathcal{C}_k} \sqrt{(w_x^i - \bar{w}_x^k)^2 + (w_y^i - \bar{w}_y^k)^2} + \frac{r}{2}$$

> **Note:** $+\frac{r}{2}$ adds half a cell as safety margin

**Final output per obstacle:**

$$\boxed{\text{obstacle}_k = \left(\bar{w}_x^k,\ \bar{w}_y^k,\ \rho_k\right) \quad \forall\ k \in [1,\ K]}$$

---

## Full Pipeline

$$\underbrace{\mathcal{G},\ \Delta\mathcal{G}}_{\text{/local\_costmap/costmap} \atop \text{/local\_costmap/costmap\_updates}}
\xrightarrow{\text{Step 1}} \underbrace{\mathcal{G}_{current}}_{\text{patched grid}}
\xrightarrow{\text{Steps 2-4}} \underbrace{\text{circle of cells}}_{\text{around robot}}
\xrightarrow{\text{Step 5}} \underbrace{\mathcal{P}}_{\text{lethal points in metres}}
\xrightarrow{\text{Step 6}} \underbrace{\mathcal{C}_1 \ldots \mathcal{C}_K}_{\text{clusters}}
\xrightarrow{\text{Step 7}} \underbrace{(\bar{w}^k,\ \rho_k)}_{\text{obstacle centre + radius}}$$






