#!/usr/bin/env python
"""Node for generating a costmap for global planning.

Subscribed Topics:
   map - (nav_msgs/OccupancyGrid) Standard ROS occupancy grid

Published Topics:
   costmap - (nav_msgs/OccupancyGrid)
     Costmap cell values may be:
        -1    - unknown
         100  - blocked
         1-99 - cost to traverse

Parameters:
  robot_radius - (float, default: .18)
     Radius of the robot for the purpose of obstacle dilation.

  ~sigma - (float, default: .1)
     Parameter determining fall-off rate of cost penalty for passing
     near blocked cells. Larger values result in more conservative
     navigation.

"""
import numpy as np
import rospy
import map_utils
import cv2
from nav_msgs.msg import OccupancyGrid

class CostmapMaker(object):
    """
    Node for creating a costmap from an occupancy grid.
    """

    def __init__(self):
        """ Initialize the particle.  """
        rospy.init_node('costmap_node')

        rospy.Subscriber("map", OccupancyGrid, self.map_callback)

        self.costmap_pub = rospy.Publisher('costmap', OccupancyGrid,
                                           latch=True, queue_size=10)

        self.robot_radius = rospy.get_param('robot_radius', .18)
        self.sigma = rospy.get_param('~sigma', .1)

        rospy.spin()

    def map_callback(self, map_msg):
        """ Create and publish the costmap. """
        world_map = map_utils.Map(map_msg)
        cost_map = map_utils.Map(map_msg)
        # first inflate the obstacles...
        dilation_size = int(self.robot_radius / world_map.resolution) * 2 + 1
        rospy.loginfo("{}".format(dilation_size))

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (dilation_size,
                                                               dilation_size))
        grid_occupied = np.zeros(world_map.grid.shape)
        occupied_indices = world_map.grid == 100
        grid_occupied[occupied_indices] = 1
        rospy.loginfo("dilating costmap...")
        grid_dilated = cv2.dilate(grid_occupied, kernel)

        indices = np.logical_and(grid_dilated == 1, world_map.grid != -1)

        # fully dilated grid...
        cost_map.grid[grid_dilated == 1] = 100

        rospy.loginfo('building KDTree')
        from sklearn.neighbors import KDTree
        occupied_points = []
        all_positions = []
        for i in range(cost_map.grid.shape[0]):
            for j in range(cost_map.grid.shape[1]):
                all_positions.append(cost_map.cell_position(i, j))
                if cost_map.grid[i, j] == 100:
                    occupied_points.append(cost_map.cell_position(i, j))

        kdt = KDTree(occupied_points)

        dists = kdt.query(all_positions, k=1)[0][:]
        probs = np.exp(-(dists**2) / (2 * self.sigma**2))

        dist_costs = probs.reshape(cost_map.grid.shape) * 100
        dist_costs = np.array(dist_costs, dtype='int8')

        indices = np.logical_and(dist_costs > cost_map.grid,
                                 dist_costs > 0)
        cost_map.grid[indices] = dist_costs[indices]

        # no cell should have zero cost...
        cost_map.grid[cost_map.grid == 0] = 1

        # import matplotlib.pyplot as plt
        # plt.imshow(world_map.grid,interpolation='none')
        # plt.show()

        rospy.loginfo("publishing costmap...")
        self.costmap_pub.publish(cost_map.to_message())

if __name__ == "__main__":
    CostmapMaker()
