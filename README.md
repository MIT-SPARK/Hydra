## Kimera-DSG

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

cd src
git clone git@github.mit.edu:SPARK/Kimera-DSG-Builder kimera_dsg_builder --recursive
wstool init
wstool merge kimera_dsg_builder/install/kimera_scene_graph.rosinstall
# Kimera-VIO and Kimera-VIO-ROS are required, but you can just point them to the right branch (feature/online_dsg) if you have them in your workspace
# wstool merge kimera_dsg_builder/install/vio_overlay.rosinstall
wstool up

rosdep install --from-paths src --ignore-src -r -y

catkin build
```

:warning: If you're getting errors with nanoflann (for `kimera_topology` or `pybind11`) you may need to go manually initialize those submodules (go to the appropriate repo and run `git submodule update --init --recursive`).

File any and all build errors as an issue

### Components

See the following for more information:
  - [kimera_dsg_builder](kimera_dsg_builder/README.md)
  - [object_db](object_db/README.md)
