# mir_2dexplore

This package provides launch file for exploring unknown environment while
mapping them. This can be used to save time required for mapping arenas in
competitions.

## Requirements
- `mir_2dslam` (for gmapping)
- `mir_2dnav` (for move\_base config files)
- [`explore_lite`](http://wiki.ros.org/explore_lite)
- [`frontier_exploration`](http://wiki.ros.org/frontier_exploration)

## Usage

- To use `explore_lite` after bringup
  - `roslaunch mir_2dexplore explore_lite.launch` (NOTE: This will start moving
    the robot as soon as launched. Make sure that robot is within the arena.)

- To use `frontier_exploration` after bringup
  - `roslaunch mir_2dexplore frontier_exploration.launch`
  - Create a polygon in rviz using "publish point" (NOTE: `exploration_polygon_marker`
    needs to be visualised to see the created polygon (Displays -> Add -> By
    topic -> frontier_exploration -> Marker -> exploration_polygon_marker))
  - Click using "publish point" within the created polygon to start exploration
