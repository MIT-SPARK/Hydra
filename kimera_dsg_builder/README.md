# Kimera DSG-Builder

## Running online

Steps to running with ground truth:

  1. Grab the uhumans2 dataset (if you haven't already). You can probably just use the office scene without any humans (`uHumans2_office_s1_00h.bag`)

  2. Start up ROS (allows Rviz to persist between runs)

    roscore

  3. Start up the DSG builder (starting this before rviz makes sure `use_sim_time` is set properly)

    roslaunch kimera_dsg_builder uhumans2_incremental_dsg.launch

  4. Start rviz:

    roscd kimera_dsg_builder
    rviz -d rviz/uhumans2_backend.rviz

  5. Start the bag:

    cd /path/to/uhumans2/office/scene
    rosbag play uHumans2_office_s1_00h.bag --clock --pause -r 0.5

You way want to try the following arguments for the `uhumans2_incremental_dsg` launch file:

  - `use_gt_frame:=false` switches the code to listen for the TF from Kimera-VIO
  - `min_glog_level:=0` will enable helpful development output.
  - `verbosity:=3` will enable a lot more development output
  - `debug:=true` will run the dsg builder in gdb (hopefully not necessary)

You can find parameters for the front-end [here](config/dsg_frontend_config.yaml)
and for the back-end [here](config/dsg_backend_config.yaml)

Important parameters:

  - `pgmo/*`: all pgmo settings for the `MeshFrontend` and `KimeraPgmoInterface` (frontend and backend configs respectively)
  - `dsg/add_places_to_deformation_graph`: controls place-based factors
  - `dsg/optimize_on_lc`: allows disabling optimization

## Running offline

:warning: very preliminary and incomplete documentation

Running this:

```
roslaunch kimera_dsg_builder kimera_dsg_builder_goseek.launch
```

will get you both the offline builder and the visualizer. It requires at least a tsdf put in the right location (see launch files for details).

---

# Legacy Readme

*TODO(nathan): remove*

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
