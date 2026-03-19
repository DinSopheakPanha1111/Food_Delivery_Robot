#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading

import rclpy
from rclpy.node import Node
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid


# ── Shared state ─────────────────────────────────────────────────────────────
grid  = None
glock = threading.Lock()
dirty = [False]


class CostmapVisualizer(Node):
    def __init__(self):
        super().__init__("costmap_visualizer")
        self.create_subscription(OccupancyGrid,       "/local_costmap/costmap",         self.cb_full,   10)
        self.create_subscription(OccupancyGridUpdate, "/local_costmap/costmap_updates",  self.cb_update, 10)
        self.get_logger().info("Subscribed to /local_costmap/costmap and /local_costmap/costmap_updates")

    def cb_full(self, msg):
        global grid
        with glock:
            grid = np.array(msg.data, dtype=np.int8).reshape(
                (msg.info.height, msg.info.width)
            )
            dirty[0] = True

    def cb_update(self, msg):
        global grid
        with glock:
            if grid is None:
                return
            patch = np.array(msg.data, dtype=np.int8).reshape((msg.height, msg.width))
            grid[msg.y : msg.y + msg.height, msg.x : msg.x + msg.width] = patch
            dirty[0] = True


def ros_spin(node):
    rclpy.spin(node)


# ── ROS 2 init ────────────────────────────────────────────────────────────────
rclpy.init()
node = CostmapVisualizer()
spin_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
spin_thread.start()


# ── Plot setup ────────────────────────────────────────────────────────────────
cmap = plt.cm.RdYlGn_r.copy()
cmap.set_under("#404040")   # unknown cells (-1)

fig, ax = plt.subplots(figsize=(7, 7))
fig.canvas.manager.set_window_title("local_costmap")

im = ax.imshow(
    np.zeros((10, 10), dtype=np.int8),
    cmap=cmap, vmin=0, vmax=100,
    interpolation="nearest", origin="lower"
)
fig.colorbar(im, ax=ax, label="Cost  (0 = free,  100 = occupied)")
ax.set_title("/local_costmap/costmap_updates")


def update(_):
    with glock:
        if dirty[0] and grid is not None:
            im.set_data(grid)
            im.set_extent([0, grid.shape[1], 0, grid.shape[0]])
            ax.set_xlim(0, grid.shape[1])
            ax.set_ylim(0, grid.shape[0])
            dirty[0] = False
    return [im]


ani = animation.FuncAnimation(fig, update, interval=100, blit=True, cache_frame_data=False)
plt.tight_layout()
plt.show()

node.destroy_node()
rclpy.shutdown()