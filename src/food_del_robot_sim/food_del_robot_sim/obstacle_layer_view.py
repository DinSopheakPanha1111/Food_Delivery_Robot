#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.msg import Costmap
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import numpy as np

class CostmapPlotter(Node):
    def __init__(self):
        super().__init__('costmap_plotter')
        self.subscription = self.create_subscription(
            Costmap,
            '/global_costmap/costmap_raw',
            self.costmap_callback,
            10
        )
        self.get_logger().info('Waiting for costmap...')

    def costmap_callback(self, msg):
        width = msg.metadata.size_x
        height = msg.metadata.size_y
        data = np.array(msg.data, dtype=np.float32).reshape((height, width))

        # Nav2 costmap color scheme
        # 0   = free (white)
        # 1-252 = inflation gradient (blue to red)
        # 253 = inscribed (red)
        # 254 = lethal obstacle (black)
        # 255 = unknown (grey)

        cmap = mcolors.ListedColormap([
            '#ffffff',  # 0: free
            *plt.cm.Blues(np.linspace(0.2, 0.8, 127)),  # 1-127: low inflation
            *plt.cm.Reds(np.linspace(0.2, 0.8, 125)),   # 128-252: high inflation
            '#ff0000',  # 253: inscribed
            '#000000',  # 254: lethal
            '#808080',  # 255: unknown
        ])

        bounds = list(range(257))
        norm = mcolors.BoundaryNorm(bounds, cmap.N)

        plt.figure(figsize=(10, 10))
        plt.imshow(data, cmap=cmap, norm=norm, origin='lower')
        cbar = plt.colorbar(ticks=[0, 1, 127, 252, 253, 254, 255])
        cbar.ax.set_yticklabels(['Free', 'Low Inflate', 'Mid Inflate', 'High Inflate', 'Inscribed', 'Lethal', 'Unknown'])
        plt.title('Global Costmap (costmap_raw)')
        plt.xlabel('X (cells)')
        plt.ylabel('Y (cells)')
        plt.tight_layout()
        plt.savefig('global_costmap.png', dpi=150)
        plt.show()
        self.get_logger().info('Costmap saved as global_costmap.png')
        rclpy.shutdown()

def main():
    rclpy.init()
    node = CostmapPlotter()
    rclpy.spin(node)

main()