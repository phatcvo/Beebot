  # emcl_ros

ROS implementation of emcl (mcl with expansion resetting)

Support Dynamic Reconfigure

<p align="center">
  <img src="images/rqt_reconfigure.png" height="360px"/>
</p>

## How to use

```
roslaunch emcl_ros emcl.launch
```

## Running the demo

```
# run demo
## terminal 1
export TURTLEBOT3_MODEL=burger
roslaunch emcl_ros test.launch
## terminal 2
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

### Use gyrodometry

```
roslaunch emcl_ros test.launch use_gyrodom:=true
```

## Node I/O

![Node I/O](images/node_io.png)

## Nodes

### emcl

#### Published Topics

- /emcl_pose (`geometry_msgs/PoseWithCovarianceStamped`)
  - The estimated pose of the robot
- /tf (`tf2_msgs/TFMessage`)
  - tf (from the global frame to the odom frame)
- ~\<name>/particle_cloud (`geometry_msgs/PoseArray`)
  - The particle cloud of mcl

#### Subscribed Topics

- /cloud (`sensor_msgs/PointCloud2`)
  - The input pointcloud data
  - If ~\<name>/use_cloud is false, this topic is not used
- /initialpose (`geometry_msgs/PoseWithCovarianceStamped`)
  - The initial pose of the robot
- /odom (`nav_msgs/Odometry`)
  - The odometry data
- /scan (`sensor_msgs/LaserScan`)
  - The input laser scan data (default)

#### Parameters

##### EMCL Parameters

- ~\<name>/<b>expansion_position_dev</b> (float, default: `0.07` [m]):<br>
  The standard deviation of the expansion noise in position
- ~\<name>/<b>expansion_orientation_dev</b> (float, default: `0.2` [rad]):<br>
  The standard deviation of the expansion noise in orientation
- ~\<name>/<b>init_x</b> (float, default: `0.0` [m]):<br>
  The initial x position of the robot
- ~\<name>/<b>init_y</b> (float, default: `0.0` [m]):<br>
  The initial y position of the robot
- ~\<name>/<b>init_yaw</b> (float, default: `0.0` [rad]):<br>
  The initial yaw of the robot
- ~\<name>/<b>init_position_dev</b> (float, default: `0.1` [m]):<br>
  The standard deviation of the initial noise in position
- ~\<name>/<b>init_orientation_dev</b> (float, default: `0.05` [rad]):<br>
  The standard deviation of the initial noise in orientation
- ~\<name>/<b>laser_step</b> (int, default: `4`):<br>
  The step of the laser scan
- ~\<name>/<b>likelihood_th</b> (float, default: `0.002`):<br>
  The threshold of the likelihood
- ~\<name>/<b>particle_num</b> (int, default: `420`):<br>
  The number of particles
- ~\<name>/<b>reset_count_limit</b> (int, default: `3`):<br>
  The limit of the reset count
- ~\<name>/<b>sensor_noise_ratio</b> (float, default: `0.03`):<br>
  The ratio of sensor noise to the actual sensor noise
- ~\<name>/<b>use_cloud</b> (bool, default: `False`):<br>
  If true, use pointcloud instead of laser scan

##### OdomModel Parameters

- ~\<name>/<b>ff</b> (float, default: `0.17` [m]):<br>
  Standard deviation of forward noise per forward
- ~\<name>/<b>fr</b> (float, default: `0.0005` [m]):<br>
  Standard deviation of forward noise per rotation
- ~\<name>/<b>rf</b> (float, default: `0.13` [rad]):<br>
  Standard deviation of rotation noise per forward
- ~\<name>/<b>rr</b> (float, default: `0.2` [rad]):<br>
  Standard deviation of rotation noise per rotation

##### Scan Parameters

If pointcloud is used, following parameters are used.

- ~\<name>/<b>range_min</b> (float, default: `0.12` [m]):<br>
  The minimum range of the sensor
- ~\<name>/<b>range_max</b> (float, default: `3.5` [m]):<br>
  The maximum range of the sensor

##### Common Parameters

- ~\<name>/<b>use_dynamic_reconfigure</b> (bool, default: `False`):<br>
  If true, use dynamic reconfigure

## References

- https://github.com/ryuichiueda/emcl
- https://github.com/NaokiAkai/ALSEdu
