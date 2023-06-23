## Hydra-ROS

Contains ROS code specific to Hydra.

To build and run tests (assuming you've built the package normally):

```
catkin build hydra_utils --catkin-make-args tests
rostest hydra_utils hydra_utils.test
```
