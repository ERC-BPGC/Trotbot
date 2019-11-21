# Navigation Package

## Usage:

Meet the prequisites mentioned [here](https://github.com/ERC-BPGC/Trotbot).

Then:
```
# To use the path planner
rosrun navigation path_planner.py
```

## Hardware Used:
- RP-Lidar A1
    - The laserscan from RP-Lidar A1 is used for obstacle detection.
- Intel Realsense D435i
    - Realsense is used to get the odometry using [rovio](https://github.com/ethz-asl/rovio).


## Known Issues:

## Results:
Path Planning:


<img src="https://github.com/ERC-BPGC/Trotbot/blob/master/navigation/scripts/rrt_for_scan/tests/plan.png" alt="RRT" title="RRT" width="400"/> <img src="https://github.com/ERC-BPGC/Trotbot/blob/master/navigation/scripts/rrt_star/tests/RRTstar_without_max_iter.png" alt="RRT*" title="RRT*" width="400"/>
