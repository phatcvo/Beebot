# likelihood_field_gridmap_ros

ROS implementation of Likelihood Field Grid Map

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
