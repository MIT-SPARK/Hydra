## Hydra Utilities

Contains common code used between the different hydra packages, which primarily consists of configuration parsing.

To build and run tests (assuming you've built the package normally):

```
catkin build hydra_utils --catkin-make-args tests
rostest hydra_utils hydra_utils.test
```

Documentation:
-  [Config Parsing](doc/config_parsing.md)
