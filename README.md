# graph_slam

A ROS 2 (`ament_cmake`) package implementing graph-based SLAM for cone observations, built on [g2o](https://github.com/RainerKuemmerle/g2o) (SE2 pose graph) with PCL-accelerated landmark association and mission-aware map persistence.

## Overview

The main node, [`GraphSLAM_Node`](graph_slam/include/graph_slam/graph_slam_node.hpp), wraps the solver class [`GraphSLAM`](graph_slam/include/graph_slam/graph_slam.hpp) (implemented in [graph_slam/src/graph_slam.cpp](graph_slam/src/graph_slam.cpp)).

### Frame policy — base frame = REAR AXLE

The estimator integrates and **publishes the rear-axle pose**. The dead-reckoning input `v`
(inverter / rear-wheel speed) is native to the rear axle, which has ~zero lateral velocity, so the
pure unicycle in `compute_predicted_pose()` is a correct rear-axle estimator (yaw rate `omega` is
position-independent). **Pure pursuit consumes the rear-axle pose directly.**

Cone observations on `/mapping/cones` must arrive **already in the rear-axle base frame** —
`graph_slam` applies no sensor/camera extrinsic itself. That transform is an upstream intake
concern owned by whichever bridge publishes `/mapping/cones`: `ZED_Bridge` on the real car, and
`lart_to_pacsim_bridge`'s `sensor_to_base.{x,y,yaw}` params in sim. Applying it in `graph_slam` too
would double-apply it on the real car (an earlier revision tried this — `camera_extrinsic_{x,y,yaw}_`
— and was reverted once the bridge-side contract was confirmed; see
`claude_code_pacsim_bridge_extrinsic_prompt.md`). In PacSim the front sensor sits at the CoG
(`perception.yaml` pose `[0,0,0]`), so the sim bridge's offset numerically equals the CoG-to-rear
distance `lr` (0.6975 m) — physically **distinct** from the eval harness's `--gt-ref` transform
(which uses the same `lr` for a different purpose: reconciling GT's CoG reference against a
rear-axle pose) even though the two values coincide — keep them apart.

Ground-truth reference-point conventions (PacSim reports GT at the **CoG**) are an **eval** concern
handled in the harness (`run_eval --gt-ref rear|cog`), never in the estimator. A CoG pose, if ever
needed downstream, is an output-side transform at publish time — never a term in the motion model.
(An earlier CoG side-slip DR term `v_y=lr*omega` was reverted: it only rebased the estimate onto the
CoG to match CoG-referenced GT — a reference-point change proven identical to 0.00 mm, not an
estimation fix.)

### Subscribed topics

| Topic | Message | Purpose |
|---|---|---|
| `/mapping/cones` | `lart_msgs::msg::ConeArray` | Cone observations to associate/insert into the graph |
| `/acu_origin/dynamics` | `lart_msgs::msg::Dynamics` | Wheel speed, used for dead-reckoning pose prediction |
| `/imu/angular_velocity` | `geometry_msgs::msg::Vector3Stamped` | Angular velocity, used for dead-reckoning pose prediction |
| `/pc_origin/system_status/critical_as/mission` | `lart_msgs::msg::Mission` | Current mission, drives map load/save and lap-counting behavior |

### Published topics

| Topic | Message | Purpose |
|---|---|---|
| `/slam/map` | `lart_msgs::msg::ConeArray` | Current landmark map |
| `/slam/map/markers` | `visualization_msgs::msg::MarkerArray` | Map landmarks for RViz |
| `/slam/pose` | `geometry_msgs::msg::PoseStamped` | Estimated vehicle pose |
| `/slam/stats` | `lart_msgs::msg::SlamStats` | SLAM performance/diagnostics |

A `map -> base_link` (or equivalent) transform is also broadcast via `tf2_ros::TransformBroadcaster`.

### Pipeline

1. Pose is predicted between optimizations using wheel speed + IMU angular velocity (`compute_predicted_pose`).
2. Each incoming `ConeArray` is handed to the [`AssociationSolver`](graph_slam/include/graph_slam/associationSolver.hpp) to match observations against existing landmarks (or, once a map is loaded/built, against a PCL KD-tree of the map for fast localization).
3. New SE2 pose vertices/edges and pose-landmark (`EdgeSE2PointXY`) edges are added to the g2o `SparseOptimizer`, which is periodically re-optimized (`update_graph`).
4. Lap completion is tracked from pose/distance margins; behavior (when to switch to localization-only mode, when to persist the map) depends on the active `lart_msgs::msg::Mission`.

## Association solver

Data association is handled by [`AssociationSolver`](graph_slam/include/graph_slam/associationSolver.hpp) (implemented in [graph_slam/src/associationSolver.cpp](graph_slam/src/associationSolver.cpp)), a thin front-end owning a polymorphic `AssociationBackend`:

- Mode `0` — `NearestNeighborBackend`: simple Euclidean nearest-neighbor matching.
- Mode `1` — `MahalanobisBackend`: Mahalanobis-distance matching using landmark covariance.
- Mode `2` — `ICPBackend`: PCL-based ICP matching (color classes still respected).

The active mode is selected via `ASSOCIATION_MODE` in [`graph_slam.hpp`](graph_slam/include/graph_slam/graph_slam.hpp) (currently `1`, Mahalanobis).

## Map persistence

[`MapManager`](graph_slam/include/graph_slam/map_manager.hpp) (implemented in [graph_slam/src/map_manager.cpp](graph_slam/src/map_manager.cpp)) loads and saves the g2o graph as YAML:

- On `SKIDPAD` missions, the node loads the bundled default map [`maps/skidpad.yaml.default`](maps/skidpad.yaml.default) and switches straight to localization mode.
- On `AUTOCROSS` / `TRACKDRIVE` missions, the built map is saved to `maps/mission_<id>_<timestamp>_map.yaml` once the first lap completes (switching to localization mode) and again on node shutdown.
- Saved/loaded maps are installed to `share/graph_slam/maps` at build time; see [maps/](maps) for accumulated session maps.

## Building

This is an `ament_cmake` ROS 2 package. From the root of your ROS 2 workspace (one level above `graph_slam/`):

```sh
colcon build --packages-select graph_slam
source install/setup.bash
```

### Dependencies

From [graph_slam/package.xml](graph_slam/package.xml) and [graph_slam/CMakeLists.txt](graph_slam/CMakeLists.txt):

- `rclcpp`, `lart_msgs`, `geometry_msgs`
- `tf2`, `tf2_ros`, `tf2_geometry_msgs`, `visualization_msgs`
- `yaml-cpp`, `ament_index_cpp`
- `libg2o-dev` (core, types_slam2d, solver_eigen, stuff), `fmt`
- `PCL` (registration, common, kdtree)

## Running the node

After building and sourcing your workspace:

```sh
ros2 run graph_slam graph_slam_node
```

Or via the launch files in [launch/](launch):

```sh
ros2 launch graph_slam graph_slam.launch.py
# or
ros2 launch graph_slam graph_slam.launch.xml
```

## Tools

Standalone scripts in [tools/](tools) for offline inspection of saved g2o graphs/maps (some contain hardcoded developer paths to a `.g2o` dump — edit the path at the top of the script before running):

- [tools/viewer.py](tools/viewer.py) — Matplotlib viewer for a g2o graph (poses, landmarks, odometry/observation edges) overlaid on the FSG 2025 ground-truth map.
- [tools/viewer_sim.py](tools/viewer_sim.py) — Same as above, tailored for the simulator's ground-truth track format.
- [tools/map_comparer.py](tools/map_comparer.py) — PyQt5 GUI to drag-and-drop a ground-truth map and a SLAM map, align them with ICP, and visualize/compute ATE (absolute trajectory error) between them.
