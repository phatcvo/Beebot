# likelihood_field_gridmap_ros

![Build Status](https://github.com/phatcvo/likelihood_field_gridmap_ros/workflows/build/badge.svg)

ROS implementation of Likelihood Field Grid Map

<p align="center">
  <img src="images/raw_gridmap.png" width="400"/>
  <img src="images/likelihood_field_gridmap.png" width="378"/>
</p>

## Environment

- Ubuntu 20.04
- ROS Noetic

## Install and Build

```
# clone repository
cd /path/to/your/catkin_ws/src
git clone https://github.com/phatcvo/likelihood_field_gridmap_ros.git

# build
cd /path/to/your/catkin_ws
rosdep install -riy --from-paths src --rosdistro noetic              # Install dependencies
catkin build likelihood_field_gridmap_ros -DCMAKE_BUILD_TYPE=Release # Release build is recommended
```

## Running the demo

```
roslaunch likelihood_field_gridmap_ros test.launch
```

## Node I/O

![Node I/O](images/node_io.png)

## Nodes

### likelihood_field_gridmap

#### Published Topics

- /map/likelihood_field (`nav_msgs/OccupancyGrid`):<br>
  - The gridmap with likelihood field

#### Parameters

- ~\<name>/<b>hz</b> (int, default: `1` [Hz]):<br>
  The rate of publishing the likelihood field
- ~\<name>/<b>likelihood_range</b> (float, default: `1.0` [m]):<br>
  The range of the likelihood field
