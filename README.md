## ROS 2 workspace overview

This workspace contains two ROS 2 packages:

- `Automate`: publishes the TF chain (static frames, odom, map).
- `Navigation`: generates the map and local costmap from point clouds.

## Packages and nodes

### Automate

Nodes (executables):

- `static_tf`: publishes static transforms from `base_link` to sensor frames.
- `odometry`: computes odom from point cloud and optional IMU.
- `map_tf`: publishes map->odom transform using scan matching.

Launch:

- `ros2 launch Automate launch.py`

Parameters:

- `Automate/config/params.yaml`

Key parameters:

- `static_tf -> child_frames`: list of sensor frames.
- `static_tf -> parent_frame`: typically `base_link`.
- `odometry  -> pcl2_topic`: point cloud topic.
- `odometry  -> use_imu`: enable IMU fusion.
- `odometry  -> imu_topic`: IMU topic.
- `map_tf    -> odom_topic`: odom topic.
- `map_tf    -> scan_match_topic`: scan match pose topic.

### Navigation

Nodes (executables):

- `occupancy_grid`: builds a 2D occupancy grid from point clouds.
- `local_costmap`: builds a local costmap from point clouds.

Launch:

- `ros2 launch Navigation launch.py`

Parameters:

- `Navigation/config/params.yaml`

Key parameters:

- `occupancy_grid -> pcl2_topic`: point cloud topic.
- `occupancy_grid -> map_frame`: map frame name.
- `occupancy_grid -> resolution`: grid resolution (m).
- `occupancy_grid -> min_height`: min point height.
- `occupancy_grid -> max_height`: max point height.
- `local_costmap  -> pcl2_topic`: point cloud topic.
- `local_costmap  -> map_frame`: either use map_frame or base_link frame for costmap

## Typical flow

1. Launch `Automate` to create a complete TF chain: static sensor frames -> `base_link` -> `odom` -> `map`.
2. Launch `Navigation` to build maps and costmaps using the established TF tree.

## Build (colcon)

From the workspace root:

```bash
colcon build
```

Then source:

```bash
source install/setup.bash
```
