# Trotbot

Find scripts in trotbot_ws/src/navigation

Prerequisites:
- ROS Kinetic
- Python catkin-tools


To use this workspace do:

```
cd trotbot_ws
catkin build
source devel/setup.bash
```

## TODO

- [X] Transfer dynamic parameters from class init to Planning()
- [X] Remove do_rrt()
- [X] Test new RRT code
- [X] Add animation feature to RRT code (just add plt.show() at the end of your script)
- [X] Update los_optimizer
- [ ] Using some kind of python collection for storing path (05/11/19)
- [ ] Use namedtuple() for coordinates
- [ ] Implement adjustable sample_area
- [X] Transform path in odom update
- [ ] Modify dynamic_caller to call any type of algorithm
- [X] Make skeleton ROS code for each node
- [ ] Write Controller Action server
