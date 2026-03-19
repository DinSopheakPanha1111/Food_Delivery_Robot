#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation

# ─────────────────────────────────────────────
#  Costmap parameters
# ─────────────────────────────────────────────
RESOLUTION = 0.05
ORIGIN_X   = -5.0
ORIGIN_Y   = -5.0
WIDTH      = 200
HEIGHT     = 200
V_MAX      = 0.5
T_PREDICT  = 4.0
R_METRES   = V_MAX * T_PREDICT   # 2.0 m

# ─────────────────────────────────────────────
#  Build costmap
# ─────────────────────────────────────────────
def make_costmap():
    grid = np.zeros((HEIGHT, WIDTH), dtype=np.int8)
    grid[0, :]  = 100;  grid[-1, :] = 100
    grid[:, 0]  = 100;  grid[:, -1] = 100
    grid[108:116, 108:116] = 100
    grid[95:100,  120:132] = 100
    grid[130:136, 80:86]   = 100
    grid[120:124, 95:102]  = 100
    grid[85:90,   85:95]   = 100
    grid[140:148, 130:138] = 100
    return grid

GRID = make_costmap()

# ─────────────────────────────────────────────
#  World <-> Cell
# ─────────────────────────────────────────────
def world_to_cell(wx, wy):
    cx = int((wx - ORIGIN_X) / RESOLUTION)
    cy = int((wy - ORIGIN_Y) / RESOLUTION)
    return cx, cy

def cell_to_world(cx, cy):
    wx = ORIGIN_X + (cx + 0.5) * RESOLUTION
    wy = ORIGIN_Y + (cy + 0.5) * RESOLUTION
    return wx, wy

# ─────────────────────────────────────────────
#  Bounding box + circle filter
# ─────────────────────────────────────────────
def optimised_scan(robot_wx, robot_wy):
    robot_cx, robot_cy = world_to_cell(robot_wx, robot_wy)
    r_cells = int(np.ceil(R_METRES / RESOLUTION))
    lethal_pts  = []
    scanned_pts = []

    for dy in range(-r_cells, r_cells + 1):
        for dx in range(-r_cells, r_cells + 1):
            cx = robot_cx + dx
            cy = robot_cy + dy
            if cx < 0 or cy < 0 or cx >= WIDTH or cy >= HEIGHT:
                continue
            d = np.sqrt(dx**2 + dy**2) * RESOLUTION
            if d > R_METRES:
                continue
            scanned_pts.append(cell_to_world(cx, cy))
            if GRID[cy, cx] == 100:
                lethal_pts.append(cell_to_world(cx, cy))

    return lethal_pts, scanned_pts

# ─────────────────────────────────────────────
#  Cluster → centre + radius
# ─────────────────────────────────────────────
def find_obstacles(lethal_pts):
    if not lethal_pts:
        return []
    pts  = np.array(lethal_pts)
    used = np.zeros(len(pts), dtype=bool)
    obstacles = []
    for i in range(len(pts)):
        if used[i]:
            continue
        cluster = [i]
        used[i] = True
        for j in range(i + 1, len(pts)):
            if used[j]:
                continue
            if np.linalg.norm(pts[i] - pts[j]) <= 0.3:
                cluster.append(j)
                used[j] = True
        cluster_pts = pts[cluster]
        centre = cluster_pts.mean(axis=0)
        dists  = np.linalg.norm(cluster_pts - centre, axis=1)
        radius = dists.max() + RESOLUTION / 2
        obstacles.append({'centre': centre, 'radius': radius})
    return obstacles

# ─────────────────────────────────────────────
#  Robot path
# ─────────────────────────────────────────────
t_vals = np.linspace(0, 2 * np.pi, 300)
PATH_X = 0.8 * np.cos(t_vals)
PATH_Y = 0.8 * np.sin(t_vals)

# ─────────────────────────────────────────────
#  Figure — 2 panels
# ─────────────────────────────────────────────
fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(16, 8))
fig.patch.set_facecolor('#0d0d1a')
fig.suptitle('Real-time Obstacle Detection  |  Bounding Box + Circle Filter',
             color='#e0e0ff', fontsize=13)

extent = [ORIGIN_X, ORIGIN_X + WIDTH  * RESOLUTION,
          ORIGIN_Y, ORIGIN_Y + HEIGHT * RESOLUTION]

cmap = plt.cm.RdYlGn_r.copy()
cmap.set_under('#1a1a2e')

OBS_COLORS = ['#ff9800', '#e040fb', '#40c4ff', '#ff5252', '#69f0ae', '#ffeb3b']

for ax in (ax0, ax1):
    ax.set_facecolor('#0d0d1a')
    ax.tick_params(colors='#777799')
    for sp in ax.spines.values():
        sp.set_edgecolor('#222244')

# ── Panel 0: full costmap ─────────────────────
ax0.set_title('Costmap  +  Search Area', color='#ccccee', fontsize=11)
ax0.imshow(GRID, cmap=cmap, vmin=0, vmax=100,
           origin='lower', extent=extent, interpolation='nearest', alpha=0.85)
ax0.plot(PATH_X, PATH_Y, '--', color='#444466', linewidth=1, alpha=0.5)
ax0.set_xlim(ORIGIN_X, ORIGIN_X + WIDTH  * RESOLUTION)
ax0.set_ylim(ORIGIN_Y, ORIGIN_Y + HEIGHT * RESOLUTION)
ax0.set_xlabel('World X (m)', color='#888899')
ax0.set_ylabel('World Y (m)', color='#888899')

sf0 = plt.Circle((0,0), R_METRES, color='#4fc3f7', fill=True,  alpha=0.08)
se0 = plt.Circle((0,0), R_METRES, color='#4fc3f7', fill=False, linewidth=1.5, linestyle='--')
bb0 = patches.Rectangle((0,0), 1, 1, linewidth=1.2,
                         edgecolor='#ffb74d', facecolor='none', linestyle=':')
ax0.add_patch(sf0); ax0.add_patch(se0); ax0.add_patch(bb0)
robot0, = ax0.plot([], [], 'o', color='#69f0ae', markersize=11, zorder=6)
info0   = ax0.text(0.02, 0.97, '', transform=ax0.transAxes, fontsize=9,
                   color='#ccccee', va='top',
                   bbox=dict(boxstyle='round', fc='#0d0d1a', alpha=0.85, ec='#333355'))

# ── Panel 1: CLEAN — obstacles only ──────────
# NO imshow here — pure dark background, only detected items drawn
ax1.set_title('Detected Obstacles — Centre + Radius', color='#ccccee', fontsize=11)
ax1.set_xlabel('World X (m)', color='#888899')
ax1.set_ylabel('World Y (m)', color='#888899')

# search circle
se1 = plt.Circle((0,0), R_METRES, color='#4fc3f7', fill=False,
                  linewidth=1.5, linestyle='--', alpha=0.5)
ax1.add_patch(se1)

# robot
robot1, = ax1.plot([], [], 'o', color='#69f0ae', markersize=13, zorder=8, label='Robot')

# lethal dots — fully replaced each frame
lethal_scat = ax1.scatter([], [], c='#ff4444', s=20, zorder=5,
                           alpha=0.9, label='Lethal cell (cost=100)')
ax1.legend(loc='upper left', fontsize=8,
           facecolor='#1a1a2e', labelcolor='#eeeeee', edgecolor='#333355')

# obstacle artists (max 6)
obs_fill  = []
obs_edge  = []
obs_star  = []
obs_label = []
for i in range(6):
    col = OBS_COLORS[i % len(OBS_COLORS)]
    pf  = plt.Circle((0,0), 0.001, color=col, fill=True,  alpha=0.2,  zorder=5)
    pe  = plt.Circle((0,0), 0.001, color=col, fill=False, linewidth=2.5, zorder=6)
    ps, = ax1.plot([], [], '*', color=col, markersize=15, zorder=9)
    pl  = ax1.text(0, 0, '', color=col, fontsize=9, fontweight='bold', zorder=10,
                   bbox=dict(boxstyle='round,pad=0.3', fc='#0d0d1a',
                             alpha=0.88, ec=col, linewidth=0.8))
    ax1.add_patch(pf)
    ax1.add_patch(pe)
    obs_fill.append(pf)
    obs_edge.append(pe)
    obs_star.append(ps)
    obs_label.append(pl)

obs_summary = ax1.text(0.02, 0.02, '', transform=ax1.transAxes,
                       fontsize=9, color='#e0e0ff', va='bottom',
                       bbox=dict(boxstyle='round', fc='#0d0d1a',
                                 alpha=0.88, ec='#444466'))

# ─────────────────────────────────────────────
#  Animation
# ─────────────────────────────────────────────
def update(frame):
    robot_wx = PATH_X[frame]
    robot_wy = PATH_Y[frame]

    lethal_pts, scanned_pts = optimised_scan(robot_wx, robot_wy)
    obstacles               = find_obstacles(lethal_pts)

    robot_cx, robot_cy = world_to_cell(robot_wx, robot_wy)
    r_cells = int(np.ceil(R_METRES / RESOLUTION))
    zoom    = R_METRES * 1.45

    # ── Panel 0 ──
    robot0.set_data([robot_wx], [robot_wy])
    sf0.center = se0.center = (robot_wx, robot_wy)
    bx = ORIGIN_X + (robot_cx - r_cells) * RESOLUTION
    by = ORIGIN_Y + (robot_cy - r_cells) * RESOLUTION
    bw = (2 * r_cells + 1) * RESOLUTION
    bb0.set_xy((bx, by)); bb0.set_width(bw); bb0.set_height(bw)
    info0.set_text(
        f"Robot  ({robot_wx:.2f}, {robot_wy:.2f}) m\n"
        f"R = {V_MAX} × {T_PREDICT} = {R_METRES:.1f} m\n"
        f"Cells scanned: {len(scanned_pts)}\n"
        f"Lethal pts:    {len(lethal_pts)}\n"
        f"Obstacles:     {len(obstacles)}"
    )

    # ── Panel 1 ──
    robot1.set_data([robot_wx], [robot_wy])
    se1.center = (robot_wx, robot_wy)
    ax1.set_xlim(robot_wx - zoom, robot_wx + zoom)
    ax1.set_ylim(robot_wy - zoom, robot_wy + zoom)

    # replace lethal dots completely
    if lethal_pts:
        lethal_scat.set_offsets(np.array(lethal_pts))
    else:
        lethal_scat.set_offsets(np.empty((0, 2)))

    # reset all obstacle artists
    for i in range(6):
        obs_fill[i].set_radius(0.001)
        obs_edge[i].set_radius(0.001)
        obs_star[i].set_data([], [])
        obs_label[i].set_text('')

    # draw detected obstacles
    for i, obs in enumerate(obstacles[:6]):
        cx_w, cy_w = obs['centre']
        rho        = obs['radius']
        col        = OBS_COLORS[i % len(OBS_COLORS)]
        obs_fill[i].center = obs_edge[i].center = (cx_w, cy_w)
        obs_fill[i].set_radius(rho)
        obs_edge[i].set_radius(rho)
        obs_star[i].set_data([cx_w], [cy_w])
        obs_label[i].set_position((cx_w + rho + 0.05, cy_w))
        obs_label[i].set_text(f'O{i+1}  r={rho:.2f}m')

    lines = [
        f"O{i+1}: ({o['centre'][0]:.2f}, {o['centre'][1]:.2f})  r={o['radius']:.2f}m"
        for i, o in enumerate(obstacles[:6])
    ]
    obs_summary.set_text('\n'.join(lines) if lines else 'No obstacles in range')

    return ([robot0, sf0, se0, bb0, info0,
             robot1, lethal_scat, se1, obs_summary] +
            obs_fill + obs_edge + obs_star + obs_label)


ani = animation.FuncAnimation(fig, update, frames=len(t_vals),
                               interval=50, blit=True)
plt.tight_layout()
plt.show()