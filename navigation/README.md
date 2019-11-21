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

![path planning](./scripts/rrt_for_scan/tests/plan.png "path planning")
