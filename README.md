# graph_slam

A ROS 2 package implementing a simple graph-based SLAM node using the open‑source g2o library (planned) and pluggable data association methods.

## Overview

The main node is implemented in [graph_slam/src/graph_slam.cpp](graph_slam/src/graph_slam.cpp) as class [`GraphSLAM`](graph_slam/include/graph_slam/graph_slam.hpp).  
It subscribes to a cone observation topic:

- Topic: `"/mapping/cones"` (see `CONES_TOPIC` in [`GraphSLAM`](graph_slam/include/graph_slam/graph_slam.hpp))
- Message: `lart_msgs::msg::ConeArray`

Every time a `ConeArray` is received, the node forwards the data to the association solver to associate observations with existing landmarks in the map/graph.

## Association solver

Data association is handled by [`AssociationSolver`](graph_slam/include/graph_slam/associationSolver.hpp), implemented in [graph_slam/src/associationSolver.cpp](graph_slam/src/associationSolver.cpp).

`AssociationSolver` is a thin front‑end that owns a polymorphic backend (`AssociationBackend`), similar in spirit to how solvers are selected in g2o:

- Mode `0`: Nearest‑Neighbor association (`NearestNeighborBackend`)
- Mode `1`: Mahalanobis distance‑based association (`MahalanobisBackend`)

The active mode is selected in [`GraphSLAM`](graph_slam/include/graph_slam/graph_slam.hpp) via

- `ASSICIATION_MODE` (currently set to `0`)

You can change the default association strategy by editing this macro.

## Building

This is an `ament_cmake` ROS 2 package. From the root of your ROS 2 workspace (one level above `graph_slam/`):

```sh
colcon build --packages-select graph_slam
source install/setup.bash
```

Ensure the dependencies listed in [graph_slam/package.xml](graph_slam/package.xml) are available:

- `rclcpp`
- `lart_msgs`
- `geometry_msgs`

## Running the node

After building and sourcing your workspace:

### Using ros2 run

```sh
ros2 run graph_slam graph_slam_node
```

### Using launch files

There are both Python and XML launch files in [launch/](launch):

- [launch/graph_slam.launch.py](launch/graph_slam.launch.py)
- [launch/graph_slam.launch.xml](launch/graph_slam.launch.xml)

Run either of them, for example:

```sh
ros2 launch graph_slam graph_slam.launch.py
# or
ros2 launch graph_slam graph_slam.launch.xml
```

The node will start as `graph_slam_node`, subscribe to `"/mapping/cones"`, and log the number of cones received while delegating data association to the selected backend in [`AssociationSolver`](graph_slam/include/graph_slam/associationSolver.hpp).
