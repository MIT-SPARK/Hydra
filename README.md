## Kimera-DSG-Builder

Code to build DSGs (both offline and incrementally)

### Quick Start

Requirements (you likely have them):

```
sudo apt install python-rosdep python-wstool python-catkin-tools
# if you haven't yet:
# sudo rosdep init
# rosdep update
```

To get started (YMMV):

```
mkdir -p catkin_ws/src
cd catkin_ws
catkin init
catkin config -DCMAKE_BUILD_TYPE=Release -DGTSAM_TANGENT_PREINTEGRATION=OFF -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DOPENGV_BUILD_WITH_MARCH_NATIVE=OFF
catkin config --blacklist hdf5_map_io mesh_msgs_hdf5 label_manager mesh_tools rviz_map_plugin

cd src
git clone git@github.mit.edu:SPARK/Kimera-DSG-Builder kimera_dsg_builder --recursive
wstool init
wstool merge kimera_dsg_builder/install/kimera_scene_graph.rosinstall
# Kimera-VIO and Kimera-VIO-ROS are required, so you can alternatively point them to the right branch (feature/online_dsg) if you have them in your workspace
wstool merge kimera_dsg_builder/install/vio_overlay.rosinstall
wstool up

rosdep install --from-paths src --ignore-src -r -y

catkin build
```

:warning: If you're getting errors with `nanoflann` (`kimera_topology`) or `pybind11` (`kimera_dsg_python`) you may need to go manually initialize those submodules (go to the appropriate repo and run `git submodule update --init --recursive`).

File any and all build errors as an issue

### Using a Semantic Segmentation Network

Add `semantic_recolor` to your workspace via:

```
roscd && cd ../src
wstool merge kimera_dsg_builder/install/semantic_overlay.rosinstall
```

Then, follow the instructions to install cuda and other dependencies for the `semantic_recolor` package (which can be found [here](https://github.mit.edu/SPARK/semantic_recolor_nodelet#semantic-recolor-utilities)).

### Components

See the following for more information:
  - [kimera_dsg_builder](kimera_dsg_builder/README.md)
  - [object_db](object_db/README.md)
