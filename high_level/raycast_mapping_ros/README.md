# raycast_mapping_ros

ROS implementation of Ray Casting Update Algorithm for 2D Mapping

## How to use

```
roslaunch raycast_mapping_ros raycast_mapping.launch
```

## Running the demo

```

roslaunch raycast_mapping_ros test.launch

```

## Node I/O

![Node I/O](images/node_io.png)

## Nodes

### raycast_mapping

#### Published Topics

- /local_map (`nav_msgs/OccupancyGrid`)
  - Local map data

#### Subscribed Topics

- /cloud (`sensor_msgs/PointCloud2`)
  - Input point cloud data

#### Parameters

- ~\<name>/<b>frame_id</b> (string, default: `base_footprint`):<br>
  The frame id of the local map
- ~\<name>/<b>map_reso</b> (float, default: `0.05` [m/cell]):<br>
  The resolution of the map
- ~\<name>/<b>map_size</b> (float, default: `10.0` [m]):<br>
  The size of the map
- ~\<name>/<b>yaw_reso</b> (float, default: `0.087` [rad]):<br>
  The resolution of the yaw angle for ray casting

## References

- https://myenigma.hatenablog.com/entry/20140714/1405343128

```

```
