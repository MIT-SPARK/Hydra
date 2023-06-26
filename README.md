## Hydra

<div align="center">
    <img src="doc/media/hydra.GIF">
</div>

This repository contains code to incrementally build 3D Dynamic Scene Graphs (DSGs) in real-time and is based on the papers:
  - ["Hydra: A Real-time Spatial Perception System for 3D Scene Graph Construction and Optimization"](http://www.roboticsproceedings.org/rss18/p050.pdf)
  - ["Foundations of Spatial Perception for Robotics: Hierarchical Representations and Real-time Systems"](https://arxiv.org/abs/2305.07154)

If you find this code relevant for your work, please consider citing one or both of these papers. A bibtex entry is provided below:

```
@article{hughes2022hydra,
    title={Hydra: A Real-time Spatial Perception System for {3D} Scene Graph Construction and Optimization},
    fullauthor={Nathan Hughes, Yun Chang, and Luca Carlone},
    author={N. Hughes and Y. Chang and L. Carlone},
    booktitle={Robotics: Science and Systems (RSS)},
    pdf={http://www.roboticsproceedings.org/rss18/p050.pdf},
    year={2022},
}

@article{hughes2023foundations,
         title={Foundations of Spatial Perception for Robotics: Hierarchical Representations and Real-time Systems},
         author={Nathan Hughes and Yun Chang and Siyi Hu and Rajat Talak and Rumaisa Abdulhai and Jared Strader and Luca Carlone},
         year={2023},
         eprint={2305.07154},
         archivePrefix={arXiv},
         primaryClass={cs.RO}
}
```

#### Acknowledgements

This work was partially funded by the AIA CRA FA8750-19-2-1000, ARL DCIST CRA W911NF-17-2-0181, and ONR RAIDER N00014-18-1-2828.

#### Disclaimer

Research was sponsored by the United States Air Force Research Laboratory and the United States Air Force Artificial Intelligence Accelerator and was accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views and conclusions contained in this document are those of the authors and should not be interpreted as representing the official policies, either expressed or implied, of the United States Air Force or the U.S. Government. The U.S. Government is authorized to reproduce and distribute reprints for Government purposes notwithstanding any copyright notation herein.

## News

**Update (06/26/23):** We've released initial changes from the our newest paper.
We also plan to release additional code, most notably for training the room classification networks and GNN-based descriptors as described in the above paper.
We will link to the new repository once this is done.

:warning: As part of the this release, we have moved ROS related code to a new repository located [here](https://github.com/MIT-SPARK/Hydra-ROS). This code (and our installation process) still do rely on the ROS ecosystem.

## Installation and Running

### General Requirements

Hydra has been tested on Ubuntu 20.04 and ROS Noetic

:warning: Ubuntu 18.04 and ROS Melodic are no longer actively tested

You can follow the instructions [here](http://wiki.ros.org/ROS/Installation) to install ROS if you haven't already.
Then, make sure you have some general requirements:
```
sudo apt install python3-rosdep python3-catkin-tools python3-vcstool
```

Finally, if you haven't set up rosdep yet:
```
sudo rosdep init
rosdep update
```

### Filing Issues

:warning: We don't support other platforms. Issues requesting support on other platforms (e.g. Ubuntu 16.04, Windows) will be summarily closed.

Depending on the nature of the issue, it may be helpful to browse [this](doc/debugging.md) page about debugging Hydra first.

### Building Hydra

To get started:

```
mkdir -p catkin_ws/src
cd catkin_ws
catkin init
catkin config -DCMAKE_BUILD_TYPE=Release -DGTSAM_TANGENT_PREINTEGRATION=OFF \
              -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
catkin config --skiplist hdf5_map_io mesh_msgs_hdf5 label_manager mesh_tools \
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

### Quickstart

To test Hydra out, you can just download a single scene (the office scene without humans is recommended, and can be found [here](https://drive.google.com/uc?id=1CA_1Awu-bewJKpDrILzWok_H_6cOkGDb).
Make sure to decompress the rosbag (`rosbag decompress path/to/bagfile`) before running!

:warning: Also make sure to source the workspace before starting (typically `source path/to/catkin_ws/devel/setup.bash`, though if you use zsh you should use the correct setup file for that).

To start Hydra:
```
roslaunch hydra_ros uhumans2.launch
```

Then, start the rosbag in a separate terminal:
```
rosbag play path/to/rosbag --clock
```

### Running Hydra

See [here](https://github.com/MIT-SPARK/Hydra-ROS/blob/main/doc/quickstart.md) for detailed instructions discussing how to run Hydra using ROS.
These also detail how to use Hydra with [Kimera-VIO](https://github.com/MIT-SPARK/Kimera-VIO.git), including how to build Kimera-VIO alongside Hydra.

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

### Config Library

Hydra has a wrapper around config parsing that is documented [here](doc/config_parsing.md).


