## Hydra

<div align="center">
    <img src="doc/media/hydra.GIF">
</div>

This repository contains code to build DSGs (both offline and incrementally) and is primarily based on the paper ["Hydra: A Real-time Spatial Perception System for 3D Scene Graph Construction and Optimization"](http://www.roboticsproceedings.org/rss18/p050.pdf).  If you find this code relevant for your work, please consider citing this paper. A bibtex entry is provided below:

```
@article{hughes2022hydra,
    title={Hydra: A Real-time Spatial Perception Engine for 3D Scene Graph Construction and Optimization},
    fullauthor={Nathan Hughes, Yun Chang, and Luca Carlone},
    author={N. Hughes and Y. Chang and L. Carlone},
    booktitle={Robotics: Science and Systems (RSS)},
    pdf={http://www.roboticsproceedings.org/rss18/p050.pdf},
    year={2022},
}
```

### General Requirements

Hydra has only been tested on the following systems:

- Ubuntu 18.04 and ROS Melodic
- Ubuntu 20.04 and ROS Noetic (*recommended if possible*)

You can follow the instructions [here](http://wiki.ros.org/ROS/Installation) to install ROS if you haven't already.

Then, make sure you have some general requirements.

For melodic:
```
sudo apt install python-rosdep python-catkin-tools python3-vcstool
```

For noetic:
```
sudo apt install python3-rosdep python3-catkin-tools python3-vcstool
```

Finally, if you haven't set up rosdep yet:
```
sudo rosdep init
rosdep update
```

### Building Hydra

To get started:

```
mkdir -p catkin_ws/src
cd catkin_ws
catkin init
catkin config -DCMAKE_BUILD_TYPE=Release -DGTSAM_TANGENT_PREINTEGRATION=OFF -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DOPENGV_BUILD_WITH_MARCH_NATIVE=OFF
catkin config --blacklist hdf5_map_io mesh_msgs_hdf5 label_manager mesh_tools rviz_map_plugin minkindr_python

cd src
git clone git@github.mit.edu:SPARK/Kimera-DSG-Builder.git hydra_dsg_builder
vcs import . < hydra_dsg_builder/install/hydra.rosinstall
# vcs import . < hydra_dsg_builder/install/vio_overlay.rosinstall # (if you want to use Kimera-VIO)

rosdep install --from-paths src --ignore-src -r -y
sudo apt install libprotobuf-dev

catkin build
```

:warning: Depending on the amount of RAM available on your machine and whether or not you are compiling Kimera-VIO as well, you may run out of memory when compiling with `catkin build` directly (which will result in a `GCC killed` error). If this occurs, you can either specify fewer threads for catkin via `catkin build -j NUM_THREADS` or compile certain larger packages (e.g. gtsam) directly first by building them specifically, e.g. `catkin build gtsam`.

Please help us by creating new issues when you run into problems with these instructions!

Finally, if you'd prefer to use the older `wstool` instead of `vcs`, you can do the following instead of `vcs import`:
```
wstool init
wstool merge hydra_dsg_builder/install/hydra.rosinstall
wstool up
```

### Using a Semantic Segmentation Network

Add `semantic_recolor` to your workspace via:

```
roscd && cd ../src
vcs import . < hydra_dsg_builder/install/semantic_overlay.rosinstall
```

Then, follow the instructions to install cuda and other dependencies for the `semantic_recolor` package (which can be found [here](https://github.mit.edu/SPARK/semantic_recolor_nodelet#semantic-recolor-utilities)).

### Components

See the following for more information:
  - [hydra_dsg_builder](hydra_dsg_builder/README.md)

### Filing Issues

:warning: We don't support other platforms. Issues requesting support on other platforms (e.g. Ubuntu 16.04, Windows) will be summarily closed.
