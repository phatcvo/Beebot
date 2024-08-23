# icp_matching_ros

ROS implementation of ICP (Iterative Closest Point) matching

Provide a library `icp_matching`, demo nodes.

## Setup

```
cd icp_matching_ros
./download.sh
```

## Running the demo

### 2D

```
roslaunch icp_matching_ros test.launch
```

### 3D

```
roslaunch icp_matching_ros test.launch use_3d:=true
```

## Nodes

### icp_matching_demo

#### Published Topics

- ~\<name>/cloud_src (`sensor_msgs/PointCloud2`)
  - Source point cloud
- ~\<name>/cloud_src_registered (`sensor_msgs/PointCloud2`)
  - Registered point cloud
- ~\<name>/cloud_target (`sensor_msgs/PointCloud2`)
  - Target point cloud

#### Parameters

- ~\<name>/<b>src_pcd_path</b> (string, default: `src_pcd.pcd`):<br>
  The path to the source point cloud
- ~\<name>/<b>target_pcd_path</b> (string, default: `target_pcd.pcd`):<br>
  The path to the target point cloud
- ~\<name>/<b>frame_id</b> (string, default: `map`):<br>
  The frame id of the point cloud
- ~\<name>/<b>enable_downsampling</b> (bool, default: `false`):<br>
  Enable downsampling
- ~\<name>/<b>leaf_size</b> (float, default: `0.003`):<br>
  The leaf size of the voxel grid filter
- ~\<name>/<b>sleep_time</b> (float, default: `0.3`):<br>
  The sleep time between iterations

### test_pcd_creator

#### Parameters

- ~\<name>/<b>src_pcd_path</b> (string, default: `src_pcd.pcd`):<br>
  The path to the source point cloud
- ~\<name>/<b>target_pcd_path</b> (string, default: `target_pcd.pcd`):<br>
  The path to the target point cloud
- ~\<name>/<b>point_num</b> (int, default: `20`):<br>
  The number of points in the point cloud
- ~\<name>/<b>rotation_yaw</b> (float, default: `0.2` [rad]):<br>
  The rotation angle around the z-axis
- ~\<name>/<b>translation_x</b> (float, default: `5.0` [m]):<br>
  The translation distance along the x-axis
- ~\<name>/<b>translation_y</b> (float, default: `-2.0` [m]):<br>
  The translation distance along the y-axis

## References

- https://pcl.readthedocs.io/projects/tutorials/en/latest/interactive_icp.html
- https://docs.ros.org/en/hydro/api/pcl/html/classpcl_1_1IterativeClosestPoint.html
- https://natsutan.hatenablog.com/entry/2023/01/27/193311
