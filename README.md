## Nodes

| Node                | Method                             |
| ------------------- | ---------------------------------- |
| localizer           | emcl: mcl with expansion resetting |
| global_path_planner | A\* search algorithm               |
| local_map_creator   | Ray casting update                 |
| local_path_planner  | DWA: Dynamic Window Approach       |

## How to run

```
roslaunch system_ros demo.launch
```
