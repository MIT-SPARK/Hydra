## Setting up Hydra for a new sensor

### Requirements

This documentation is meant to be a quick guide to get you started with setting up a new launch file or configuration for Hydra.
You will need:
- An RGB-D camera
- A source of odometry (i.e., something populating the pose of the camera for every image timestamp via TF)
- A computer capable of running Nvidia TensorRT (i.e., an Nvidia GPU)

> **Note** <br>
> For the purposes of this guide (and in the code-base), we use the convention `parent_T_child` to represent the 6-DOF homogeneous transformation between from the `child` frame to the `parent` frame, or equivalently the pose of the `child` frame with respect to the `parent` frame.

Hydra does its best to follow [REP 105](https://www.ros.org/reps/rep-0105.html).  There are four coordinate frames you should be familiar with and identify for your system:
- **robot_frame**: This is typically `base_link` and serves the same purpose of REP 105; it is meant to be the root of the TF tree for the robot's body frame.
- **sensor_frame**: This is the optical frame of the camera (i.e., x right, y down, z forward).
- **odom_frame**: This is `odom` in REP 105. This serves as a reference frame for the robot and sensor poses (i.e., Hydra looks up `odom_T_robot` and `odom_T_sensor`).
- **map_frame**: This is `map` in REP 105. Hydra can optionally broadcast `map_T_odom` as the relative transform between the last optimized and unoptimized robot pose.

Finally, you will need to have set up `semantic_inference` following [these](https://github.com/MIT-SPARK/semantic_inference/blob/main/docs/closed_set.md) instructions, or have another 2D closed set dense semantic segmentation network capable of producing category labels.

### Putting together the launch file

We'll use the Zed2i camera as a working example. The camera provides the following topics:
- `/zed/zed_node/left/image_rect_color`: Color image
- `/zed/zed_node/left/camera_info`: Color camera information
- `/zed/zed_node/depth/depth_registered`: Depth image (rectified to color camera)
- `/zed/zed_node/depth/camera_info`: Depth camera information
- ...

#### Semantic Segmentation

The first step is to set up the segmentation network by including `closed_set.launch.yaml` from `semantic_inference`:
```yaml
- group:  # semantic segmentation
  - push_ros_namespace: zed
  - set_remap: {from: color/image_raw, to: zed_node/left/image_rect_color}
  - include:
      file: $(find-pkg-share semantic_inference_ros)/launch/closed_set.launch.yaml
      arg:
        - {name: labelspace_name, value: $(arg labelspace_name)}
```

There are three key points.

The first is that we namespace everything in the `zed` namespace to make connecting inputs to the node responsible for segmentation easier, and is good practice for multi-camera setups:
```yaml
- group:
  - push_ros_namespace: zed
```

The second is that we remap the input of the closed set node:
```yaml
  - set_remap: {from: color/image_raw, to: zed_node/left/image_rect_color}
```

The third is that we need to set the labelspace that `semantic_inference` uses:
```yaml
  arg:
    - {name: labelspace_name, value: $(arg labelspace_name)}
```
This remaps the output of the network (the 150 categories from ade20k) to a condense labelspace of 50ish labels.

#### Hydra

Next, we add remappings to connect the input topics for Hydra to the camera:
```yaml
- set_remap: {from: hydra/input/camera/rgb/image_raw, to: /zed/zed_node/left/image_rect_color}
- set_remap: {from: hydra/input/camera/rgb/camera_info, to: /zed/zed_node/left/camera_info}
- set_remap: {from: hydra/input/camera/depth_registered/image_rect, to: /zed/zed_node/depth/depth_registered}
- set_remap: {from: hydra/input/camera/semantic/image_raw, to: /zed/zed_node/semantic/image_raw}
```

> **Note** <br>
> We know the topics that Hydra expects for inputs based on the input name (`camera`) and receiver type (`ClosedSetImageReceiver`) that's specified in the input configuration (which we'll look at in more detail later).

We then add Hydra:
```yaml
- node:  # hydra
    pkg: hydra_ros
    exec: hydra_ros_node
    name: hydra
    args: >
      --config-utilities-yaml {map_frame: map, odom_frame: odom, robot_frame: zed_camera_link, sensor_frame: zed_left_camera_frame}
      --config-utilities-file $(find-pkg-share hydra_ros)/doc/examples/example_config.yaml
      --config-utilities-file $(find-pkg-share hydra)/config/label_spaces/$(var labelspace_name)_label_space.yaml
      --config-utilities-yaml {log_path: $(env HOME)/.hydra/$(var dataset)}
```

> **Note** <br>
> We specify configurations to Hydra with the command line features of `config_utilities`, which is documented [here](https://github.com/MIT-SPARK/config_utilities/blob/main/docs/Parsing.md#parse-from-the-command-line).

The frame IDs that we identified earlier are specified here:
```yaml
{map_frame: map, odom_frame: odom, robot_frame: zed_camera_link, sensor_frame: zed_left_camera_frame}
```

When Hydra shuts down, it will save the scene graph and various artifacts to
```yaml
{log_path: $(env HOME)/.hydra/$(var dataset)}
```

#### Visualization

Finally, we want to connect the output topic from Hydra (`hydra/backend/dsg`) to the input for the visualizer that renders the scene graph to RViz:
```yaml
- set_remap: {from: hydra_visualizer/dsg, to: hydra/backend/dsg}
- include:  # visualization
    file: $(find-pkg-share hydra_visualizer)/launch/streaming_visualizer.launch.yaml
```

The full launch file can be found [here](./examples/example_camera.launch.yaml)

### Configuring Hydra

When making the launch file for Hydra, we included this line:
```yaml
      --config-utilities-file $(find-pkg-share hydra_ros)/doc/examples/example_config.yaml
```
which points Hydra to [this](./examples/example_config.yaml) file.

This file is divided into several parts, which we'll also walk through.

#### Configuring Inputs

This block sets up the input(s) for Hydra:
```yaml
input:
  type: RosInput
  inputs:
    camera:
      receiver:
        type: ClosedSetImageReceiver
        queue_size: 30
      sensor:
        type: camera_info
        min_range: 0.4
        max_range: 5.0
        extrinsics:
          type: ros
```

In this case, we've specified a single closed set RGB-D camera (under the name `camera`), which reads intrinsic and extrinsic information from ROS (the intrinsics from a `sensor_msgs/msg/CameraInfo` message and the extrinsics from TF).

#### Configuring the Active Window and Reconstruction

This block sets up the size of the active window (an 8 meter spatial radius) and the resolution of the underlying voxel grid (10 centimeters):
```yaml
map_window:
  type: spatial
  max_radius_m: 8.0
active_window:
  volumetric_map:
    voxels_per_side: 16
    voxel_size: 0.1
    truncation_distance: 0.3
  tsdf:
    semantic_integrator:
      type: MLESemanticIntegrator
```

#### Other useful configuration points

The backend of Hydra uses "update functors" to optimize and reconcile layers of the scene graph.
These are specified in this block:
```yaml
  update_functors:
    agents: {type: UpdateAgentsFunctor}
    objects: {type: UpdateObjectsFunctor}
    surface_places: {type: Update2dPlacesFunctor, min_size: 3}
```
