# Terminology and Design

Hydra is composed of various *modules* that are responsible for some level of encapsulated functionality and run independently. Modules may have *sinks* that get called every time a module executes with useful by-products. A nominal *pipeline* in Hydra will consist of:
  - An *input* module that is responsible for receiving and packaging sensor data.
  - An *active window* module that is responsible for handling local reconstruction around the robot.
  - A *frontend* module that is responsible for constructing an odometric 3D scene graph from the local reconstruction.
  - A *backend* module that is responsible for optimizing and merging the odometric 3D scene graph.
  - A *loop closure detection* module that is responsible for detecting loop closures in the 3D scene graph.

For historical reasons, this pipeline organization is mostly specified on the ROS side.
[This](../hydra_ros/include/hydra_ros/hydra_ros_pipeline.h) file outlines the expected set of configurations necessary to get Hydra running (minus the loop closure detection module).

# Configuration

Hydra uses the `config::initContext(argc, argv, true)` function to initialize `config::Context` from command line arguments (see [here](../hydra_ros/app/hydra_node.cpp)).
See [here](https://github.com/MIT-SPARK/config_utilities/blob/main/docs/Parsing.md#parse-from-the-command-line) for details on the actual command line interface.
This results in a single collated set of YAML-backed parameters that are available for parsing.
Most parsing happens from the initialization of `hydra::HydraRosPipeline` without any namespace.

With the exception of the input configuration (discussed below), most defaults in Hydra *should* be sane enough for the pipeline to run (though this is untested).
Labelspace information is required for the objects and the 2D places to be created (which is considered part of the input configuration discussed below).
You may need to check the backend module configuration to make sure appropriate update functors (pieces of code that process the scene graph to correct it) are configured properly (see [here](https://github.com/MIT-SPARK/Hydra/blob/2d2322d88d11121108add39508c61eb319a0688e/config/datasets/uhumans2.yaml#L72)) for the scene graph to appropriately deform after optimization and contain rooms.

It may also be helpful to run Hydra with higher verbosity (i.e., set `glog_verbosity: 1` as a parameter) to see the currently parsed configuration.
If you are parsing sensor information from `sensor_msgs::msg::CameraInfo` messages or via `tf` (discussed below), you will likely need to start the dataset to let Hydra resolve this information before printing configuration information for the frontend, backend, active window, and loop closure detection.
Hydra will print out the input configuration and other global information before attempting to resolve the sensor information.
You may also find setting the parameter `print_missing: true` useful (which will output every configuration field that was not specified by YAML parameter).

Finally, you should understand how the `config_utilities` factories work (see [here](https://github.com/MIT-SPARK/config_utilities/blob/main/docs/Factories.md)).
Searching the code-base (both Hydra and Hydra-ROS) may be necessary to find all options for virtual configurations (that the code-base uses liberally).
Something like
```
grep -rnw . -e "config::RegistrationWithConfig<UpdateFunctor"
```
will identify all registrations of implementations of a particular base class (in this case, `UpdateFunctor`).
You may also find searching for `declare_config` definitions useful (to see what gets parsed). Something like
```
grep -rnw . -e "declare_config(.*) {"
```
should find all definitions.

# ROS Interaction

Hydra breaks with other packages a little and initializes an independent `rclcpp::Node` separate from the `hydra::HydraRosPipeline` main pipeline.
This node is used to initialize a `ianvs::NodeHandleFactory` entry under `hydra_ros_node` so that sinks and other elements of the pipeline can get a `ianvs::NodeHandle` object from a namespace string.
This lets us "hide" the ROS types from the constructor of a lot of ROS-specific code.
This may be subject to change if we figure out a better pattern.

# Expected Inputs

When taking input from a camera, Hydra requires at a minimum four inputs:
- A color image (by default at `hydra/SENSOR_NAME/color/image_raw`) of type `RGB8`
- A depth image (by default at `hydra/SENSOR_NAME/depth/image_rect`) of type `16UC1` or `32FC1`
- A closed-set semantic image (by default at `hydra/SENSOR_NAME/semantic/image_raw`) this is single-channel with any integer type for the pixels
- A pose (looked up by `tf`)

It is expected that you will use [semantic_inference](https://github.com/MIT-SPARK/semantic_inference) to provide closed-set semantics when simulator-provided semantics are not available.

When taking input from a LIDAR, Hydra requires two inputs:
- A pointcloud (by default at `hydra/SENSOR_NAME/pointcloud`). This should have at least point positions and colors or labels. Labels are read from the `label` or `ring` field (though it would be straightforward to make the field name configurable). Labels can be any integer type, though overflow may happen with labels with larger bit-widths than 32, as Hydra uses `uint32_t` as the internal label type.
- A pose (looked up by `tf`)

For either option, it is expected that you configure a named sensor (i.e., `SENSOR_NAME` in the topics above). This is typically a short descriptive name for the sensor in question (e.g., for uhumans2 we use `left_cam` for `SENSOR_NAME`).
You need the following minimal configuration section for this:
```yaml
input:
  type: RosInput
  inputs:
    SENSOR_NAME:
      receiver: {type: RECEIVER_TYPE}
      sensor: SENSOR_CONFIGURATION
```
where `RECEIVER_TYPE` is typically `ClosedSetImageReceiver` or `PointcloudReceiver`.
You should fill out `SENSOR_CONFIGURATION` to match the corresponding sensor type (either `Camera` or `Lidar`, see Hydra for details).
Camera sensors support populating intrinsics from `sensor_msgs::msg::CameraInfo` messages (`type: camera_info`) and extrinsics can be populated from `tf` (`extrinsics: {type: ros}`).
If configured this way, the extrinsics will look up the transform `body_T_sensor` where sensor is the frame ID associated with the camera info message and body is the frame ID associated with the robot body.
Both frame IDs can be overridden.

Hydra also (mostly) follows the [REP 105](https://www.ros.org/reps/rep-0105.html) conventions and uses the following parameters to parse frame IDs:
- `robot_frame` (defaults to `base_link`): To get the current pose, Hydra looks up `odom_frame_T_robot_frame`.
- `odom_frame` (defaults to `odom`): To get the current pose, Hydra looks up `odom_frame_T_robot_frame`. Additionally, the frontend is published with the `odom_frame` frame ID.
- `map_frame` (defaults to `map`): Hydra publishes the backend output using the `map_frame` frame ID. Hydra will also publish the current `map_frame_T_odom_frame` estimate resulting from the backend optimization.

Finally, Hydra (when used with closed-set semantics) requires information about the labels:
- Labelspace information: This consists of information about which labels correspond to which groups of concepts (e.g., objects, 2D places), as well as mappings between labels and human readable category names. The main Hydra repository has examples of what this configuration looks like (under `config/label_spaces`). This is loaded into the set of parameters.
- *(optional)* A remapping file that takes input labels to the desired output label space. An example of this can also be found in the main Hydra repository under `config/label_remappings`. This is specified by filepath under `semantic_label_remap_filepath`.
- *(optional)* A CSV file that specifies the mapping between input colors and semantic labels. This is required for v1 of uhumans2, but is being phased out.  This is specified by filepath under `semantic_colormap_file`.

# Expected Outputs

Hydra will publish the current frontend and backend scene graph under `hydra/frontend/dsg` and `hydra/backend/dsg` by default.
These will (by default) also contain the current mesh.
The *streaming* visualizer is configured appropriately by default to subscribe to one of the topics and publish markers to rviz.
You only need to remap the topics to connect them:
```yaml
set_remap: {from: hydra_visualizer/dsg, to: hydra/backend/dsg}
```

# Working example

The uhumans2 (v2) office scene is *mostly* guaranteed to work out-of-the-box.
See [here](../hydra_ros/launch/datasets/uhumans2.launch.yaml) to get a better sense for how everything is configured.
