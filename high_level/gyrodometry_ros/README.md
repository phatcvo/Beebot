# gyrodometry_ros

ROS implementation of Gyrodometry for 2D navigation

## How to use

```
rosrun gyrodometry_ros gyrodometry_node
```

## Node I/O

![Node I/O](images/node_io.png)

## Nodes

### gyrodometry

#### Published Topics

- /gyrodom (`nav_msgs/Odometry`)
  - The gyrodometry data
- /tf (`tf2_msgs/TFMessage`)
  - tf (from the odom frame to the gyrodom frame)

#### Subscribed Topics

- /imu (`sensor_msgs/Imu`)
  - The imu data
- /odom (`nav_msgs/Odometry`)
  - The odometry data

#### Parameters

- ~\<name>/<b>child_frame_id</b> (string, default: `gyrodom`):<br>
  The child frame id of the gyrodometry data

## References

- https://lilaboc.work/archives/18464338.html
- https://github.com/amslabtech/complement
