## Hydra Utilities

Contains common code used between the different hydra packages, which primarily consists of configuration parsing.

To build and run tests (assuming you've built the package normally):

```
catkin build hydra_utils --no-deps --catkin-make-args tests
rosrun hydra_utils utest_hyrda_utils
```
