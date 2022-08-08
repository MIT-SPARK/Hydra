## Hydra

<div align="center">
    <img src="doc/media/hydra.GIF">
</div>

This repository contains code to incrementally build 3D Dynamic Scene Graphs (DSGs) in real-time and is primarily based on the paper ["Hydra: A Real-time Spatial Perception System for 3D Scene Graph Construction and Optimization"](http://www.roboticsproceedings.org/rss18/p050.pdf). If you find this code relevant for your work, please consider citing this paper. A bibtex entry is provided below:

```
@article{hughes2022hydra,
    title={Hydra: A Real-time Spatial Perception System for {3D} Scene Graph Construction and Optimization},
    fullauthor={Nathan Hughes, Yun Chang, and Luca Carlone},
    author={N. Hughes and Y. Chang and L. Carlone},
    booktitle={Robotics: Science and Systems (RSS)},
    pdf={http://www.roboticsproceedings.org/rss18/p050.pdf},
    year={2022},
}
```

### Acknowledgements and Disclaimer

**Acknowledgements:** This work was partially funded by the AIA CRA FA8750-19-2-1000, ARL DCIST CRA W911NF-17-2-0181, and ONR RAIDER N00014-18-1-2828.

**Disclaimer:** Research was sponsored by the United States Air Force Research Laboratory and the United States Air Force Artificial Intelligence Accelerator and was accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views and conclusions contained in this document are those of the authors and should not be interpreted as representing the official policies, either expressed or implied, of the United States Air Force or the U.S. Government. The U.S. Government is authorized to reproduce and distribute reprints for Government purposes notwithstanding any copyright notation herein.

### General Requirements

Hydra has been tested on the following systems:

- Ubuntu 18.04 and ROS Melodic
- Ubuntu 20.04 and ROS Noetic (*recommended*)

You can follow the instructions [here](http://wiki.ros.org/ROS/Installation) to install ROS if you haven't already.

Then, make sure you have some general requirements:

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
catkin config -DCMAKE_BUILD_TYPE=Release -DGTSAM_TANGENT_PREINTEGRATION=OFF \
              -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DOPENGV_BUILD_WITH_MARCH_NATIVE=OFF
catkin config --blacklist hdf5_map_io mesh_msgs_hdf5 label_manager mesh_tools \
                          rviz_map_plugin minkindr_python

cd src
git clone git@github.com:MIT-SPARK/Hydra.git hydra
vcs import . < hydra/install/hydra.rosinstall

rosdep install --from-paths . --ignore-src -r -y
sudo apt install libprotobuf-dev protobuf-compiler

cd ..
catkin build
```

:warning: Depending on the amount of RAM available on your machine and whether or not you are compiling Kimera-VIO as well, you may run out of memory when compiling with `catkin build` directly (which will result in a `GCC killed` error). If this occurs, you can either specify fewer threads for catkin via `catkin build -j NUM_THREADS` or compile certain larger packages (e.g. gtsam) directly first by building them specifically, e.g. `catkin build gtsam`.

Please help us by creating new issues when you run into problems with these instructions!

Finally, if you'd prefer to use the older `wstool` instead of `vcs`, you can do the following instead of `vcs import`:
```
wstool init
wstool merge hydra/install/hydra.rosinstall
wstool up
```

### Running Hydra (Quickstart)

The only dataset that is supported out-of-the-box is [uHumans2](http://web.mit.edu/sparklab/datasets/uHumans2/).
To test Hydra out, you can just download a single scene (the office scene without humans is recommended, and can be found [here](https://drive.google.com/uc?id=1CA_1Awu-bewJKpDrILzWok_H_6cOkGDb).
Make sure to decompress the rosbag (`rosbag decompress path/to/bagfile`) before running!

:warning: Also make sure to source the workspace before starting (typically `source path/to/catkin_ws/devel/setup.bash`, though if you use zsh you should use the correct setup file for that).

To start Hydra:
```
roslaunch hydra_dsg_builder uhumans2_incremental_dsg.launch start_visualizer:=true
```

Then, start the rosbag in a separate terminal:
```
rosbag play path/to/rosbag --clock
```

### Using Kimera-VIO

:warning: Hydra relies on unreleased changes to Kimera-VIO to handle visual loop-closures and receive the robot pose graph. The current public version of Kimera-VIO that Hydra points to will not generate loop closures or pose graph information for Hydra, but has been tested against the `uHumans2_office_s1_00h.bag`.

You can configure your workspace to also include Kimera-VIO by:
```
roscd && cd ../src
vcs import . < hydra/install/vio_public_overlay.rosinstall

catkin build
```

To run Hydra using Kimera:

```
roslaunch kimera_vio_ros kimera_vio_ros kimera_vio_ros_uhumans2.launch

# in separate terminal
roslaunch hydra_dsg_builder uhumans2_incremental_dsg.launch \
     start_visualizer:=true \
     use_gt_frame:=false
```

To achieve the best results with Kimera-VIO, you should run the rosbag for about half a second, pause to wait for the LCD vocabulary to load, and then unpause the rosbag.

### Using a Semantic Segmentation Network

:warning: This package is not public (yet)

Add `semantic_recolor` to your workspace via:

```
roscd && cd ../src
vcs import . < hydra/install/semantic_overlay.rosinstall
```

Then, follow the instructions to install cuda and other dependencies for the `semantic_recolor` package (which can be found [here](https://github.mit.edu/SPARK/semantic_recolor_nodelet#semantic-recolor-utilities)).

Finally, build your workspace:

```
catkin build
```

### Components

See the following for more information:
  - [hydra_dsg_builder](hydra_dsg_builder/README.md)
  - [hydra_topology](hydra_topology/README.md)
  - [hydra_utils](hydra_utils/README.md)

### Filing Issues

:warning: We don't support other platforms. Issues requesting support on other platforms (e.g. Ubuntu 16.04, Windows) will be summarily closed.

Depending on the nature of the issue, it may be helpful to browse [this](doc/debugging.md) page about debugging Hydra first.
