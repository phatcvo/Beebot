# gridmap_to_pointcloud_ros

ROS package for converting gridmap to pointcloud

<p align="center">
  <img src="images/gridmap_to_pointcloud.png" height="320px"/>
</p>

## Running the demo

```
roslaunch gridmap_to_pointcloud_ros test.launch
```

## Node I/O

![Node I/O](images/node_io.png)

## Nodes

### gridmap_to_pointcloud

#### Published Topics

- /map_cloud (`sensor_msgs/PointCloud2`)
  - Pointcloud converted from gridmap

#### Parameters

- ~\<name>/<b>hz</b> (int, default: `1` [Hz]):<br>
  The rate of main loop
