tue_astar_planner_3d
====================

navfn provides a fast interpolated navigation function that can be used to create plans for a mobile base. The planner assumes a circular robot and operates on a costmap to find a minimum cost plan from a start point to an end point in a grid. The navigation function is computed with Dijkstra's algorithm, but support for an A* heuristic may also be added in the near future. navfn also provides a ROS wrapper for the navfn planner that adheres to the nav_core::BaseGlobalPlanner interface specified in
