# AMR(Autonomous Mobile Robot) System

## System

<p align="center">
  <img src="https://user-images.githubusercontent.com/82020865/186935476-fbb8fae4-c243-412c-a0be-0c5dd2163d71.png" width="640px"/>
</p>

## Nodes

| Node                | Method                             |
| ------------------- | ---------------------------------- |
| localizer           | emcl: mcl with expansion resetting |
| global_path_planner | A\* search algorithm               |
| local_map_creator   | Ray casting update                 |
| local_path_planner  | DWA: Dynamic Window Approach       |

## Dependencies

- [Roomba](https://github.com/amslabtech/Roomba)

## Installation

```
cd <YOUR_CATKIN_WS>/src
git clone --depth=1 https://github.com/phatcvo/AMR_System.git
catkin build amr_system
```

## How to run

```
roslaunch amr_system all.launch
```
