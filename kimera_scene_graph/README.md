To run the google test suite using ROS, run:

```
catkin build kimera_scene_graph --no-deps --catkin-make-args run_tests
```

Then run the rostest file:

```
rostest kimera_scene_graph test_kimera_scene_graph.test
```

**Make sure to start `roscore` in another terminal first.**
# Dynamic 3D Scene Graph

## Usage

### Loading generated map (serialized)

Use rosservice:

## Scene-Graph Serialization (Save/Load)

After running `kimera_scene_graph_server`, the scene-graph will be serialized in the `vxblx_files` location (by default).
You can then use `kimera_scene_graph_visualizer` to load the scene-graph and visualize it in Rviz.

## FAQ

For orthographic projection in rviz, use:
`git@github.com:ajshort/rviz_ortho_view_controller.git`
`https://github.com/PickNikRobotics/rviz_visual_tools`

#### ObjectDB

I'm seeing a message:
> Waiting for object database server

You need to launch object database:
```
roslaunch object_db object_db_sim.launch 
```
