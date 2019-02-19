#!/usr/bin/env python
"""A* Global path planner for the CS354 navigation library.

This node provides a planning service that will return a minimum cost
path in the eight or four-connected grid represented by the current
costmap. Path costs are computed as the Euclidean distance between all
path steps, plus the cost of all traversed cells in the costmap.

Notes:
  * The orientation of intermediate nodes in the path are arbitrary,
    but the orientation of the final node will match the goal
    orientation.
  * All poses in the provided plan will be in the map coordinate
    frame.
  * It is not assumed that the start and goal poses are provided in
    the map coordinate frame.  They will be converted to the map frame
    before the plan is constructed.


Subscribed Topics
   costmap - (nav_msgs/OccupancyGrid)
      The costmap to use for planning.

Services
  create_global_plan - (nav_msgs/GetPlan)
     This service allows the caller to request a plan.

Author: ??
Version: ??

"""

""" HONOR CODE STATEMENT... """

if __name__ == "__main__":
    print("No code yet!")
