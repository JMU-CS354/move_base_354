# CS 354 Path Planning Package

## Included Files

The following python files are included in this package.  Each individual file contains more detailed documentation. 

* costmap.py - Node that generates a global costmap that penalizes
  colliding with, or getting too close to, obstacles.
* local_planner.py - Node that sends velocity commands to steer a
  robot along a global path.
* pure_pursuit.py - Helper code used by the local planner.
* move_base.py - Node that provides a front-end to the planning
  library in the form of a SimpleActionServer.
* map_utils.py - Useful Python wrapper for ROS OccupancyGrid messages.
* util.py - Several useful utility functions for dealing with poses in tf2.
* plan_tester.py - Unit tests for global path planners.

* astar_planner.py - UNFINISHED. A global path planner. 

## Assignment

This assignment involves the following steps:

1. Complete `astar_planner.py` so that it conforms to the provided
   documentation. You are free to code your solution so that cells are
   either eight or four-connected.  Four-connected cells lead to a
   slightly simpler (and faster) implementation, but the resulting
   paths are generally not as good. (85%)
   
2. Develop a new RRT based planner named `rrt_planner.py`.  This
   planner should satisfy the same functional requirements as the A*
   planner, apart from the optimality guarantees. (7.5%)
   
3. Develop a new PRM based planner named `prm_planner.py`. (7.5%)

4. Improve at least one of your planning algorithms with a path
   smoothing algorithm that performs post-processing on discovered
   paths to reduce their overall cost (when possible).  This is
   particularly important for RRT and PRM, but it will also be useful
   for A*.  Since A* is restricted to connecting four or
   eight-connected cells, it is often possible to create a shorter
   path by creating direct edges between non-adjacent cells. (7.5%)


Notice that the total percentage for all parts is more than 100%.  The
maximumum score on the assignment is 100%.  You are welcome to do all
four parts if you want, but only three are required for full credit.

## Running the Package

The `launch` folder contains two launch files:

* move_base.launch - This launches all navigation nodes along with an
  RViz visualization.
* move_base_sim.launch - This starts gazebo and launches all of the
  navigation nodes.

You should be able to start the simulator and planning system as
follows:

`roslaunch move_base_354 move_base_sim.launch`

The launch files accept a number of command line arguments, including
a flag that can be used to select which global planner should be used:

```
# Start the planning system using the RRT planner (rrt_planner.py):
roslaunch move_base_354 move_base_sim.launch planner:=rrt
```

```
# Start the planning system using the PRM planner (prm_planner.py):
roslaunch move_base_354 move_base_sim.launch planner:=prm
```

The default is to use the A* planner. 

Once you've launched the planning system, you should be able test your
planners by selecting navigation goals in RViz, or by running
`plan_tester.py`.
