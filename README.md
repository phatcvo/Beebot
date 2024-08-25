## Nodes

| Node                | Method                             |
| ------------------- | ---------------------------------- |
| local_map_creator   | Ray casting update                 |
| localization        | emcl: mcl with expansion resetting |
| global_path_planner | A\* search algorithm               |
| local_path_planner  | DWA: Dynamic Window Approach       |

## How to run

```
export TURTLEBOT3_MODEL=burger
roslaunch system_ros demo.launch
```
