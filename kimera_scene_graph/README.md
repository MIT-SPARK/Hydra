To run the google test suite using ROS, run:

```
catkin build kimera_scene_graph --no-deps --catkin_make_args run_tests
```# Dynamic 3D Scene Graph

## Usage

### Loading generated map (serialized)

Use rosservice:


## FAQ

For orthographic projection in rviz, use:
git@github.com:ajshort/rviz_ortho_view_controller.git
https://github.com/PickNikRobotics/rviz_visual_tools

#### ObjectDB

I'm seeing a message:
> Waiting for object database server

You need to launch object database:
```
roslaunch object_db object_db_sim.launch 
```

