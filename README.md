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

**Update (05/20/23):** We recently authored the following paper ["Foundations of Spatial Perception for Robotics: Hierarchical Representations and Real-time Systems"](https://arxiv.org/abs/2305.07154).
Associated updates to Hydra from this paper will be released soon (*current target 06/09/23*).
We also plan to release additional code, most notably for training the room classification networks and GNN-based descriptors as described in the above paper.
This will likely take slightly longer (and is likely to be in a different repository, which will be linked to).
In the meantime, if you find our new paper relevant for your work, please consider also citing this paper.
A bibtex entry is provided below:
```
@article{hughes2023foundations,
         title={Foundations of Spatial Perception for Robotics: Hierarchical Representations and Real-time Systems},
         author={Nathan Hughes and Yun Chang and Siyi Hu and Rajat Talak and Rumaisa Abdulhai and Jared Strader and Luca Carlone},
         year={2023},
         eprint={2305.07154},
         archivePrefix={arXiv},
         primaryClass={cs.RO}
}
```

### Acknowledgements and Disclaimer

**Acknowledgements:** This work was partially funded by the AIA CRA FA8750-19-2-1000, ARL DCIST CRA W911NF-17-2-0181, and ONR RAIDER N00014-18-1-2828.

**Disclaimer:** Research was sponsored by the United States Air Force Research Laboratory and the United States Air Force Artificial Intelligence Accelerator and was accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views and conclusions contained in this document are those of the authors and should not be interpreted as representing the official policies, either expressed or implied, of the United States Air Force or the U.S. Government. The U.S. Government is authorized to reproduce and distribute reprints for Government purposes notwithstanding any copyright notation herein.

### General Requirements

Hydra has been tested on Ubuntu 20.04 and ROS Noetic

:warning: Ubuntu 18.04 and ROS Melodic are no longer actively tested

You can follow the instructions [here](http://wiki.ros.org/ROS/Installation) to install ROS if you haven't already.
Then, make sure you have some general requirements:
```
sudo apt install python3-rosdep python3-catkin-tools python3-vcstool

# for melodic: sudo apt install python-rosdep python-catkin-tools python3-vcstool
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
roslaunch hydra_dsg_builder_ros uhumans2.launch start_visualizer:=true
```

Then, start the rosbag in a separate terminal:
```
rosbag play path/to/rosbag --clock
```

### Using Kimera-VIO

:warning: Kimera-VIO functionality is in a pre-release state. Support is limited (and functionality may be brittle).

You can configure your workspace to also include Kimera-VIO by:
```
roscd && cd ../src
vcs import . < hydra/install/vio_public_overlay.rosinstall

catkin build
```

#### Running Using VIO Only

First, start Kimera:

```
roslaunch kimera_vio_ros kimera_vio_ros kimera_vio_ros_uhumans2.launch online:=true viz_type:=1 use_lcd:=false
```

and in a separate terminal, run:

```
# in separate terminal
roslaunch hydra_dsg_builder_ros uhumans2.launch \
     start_visualizer:=true \
     use_gt_frame:=false
```

#### Running Using VIO and External Visual Loop Closures

First, start Kimera:

```
roslaunch kimera_vio_ros kimera_vio_ros kimera_vio_ros_uhumans2.launch online:=true viz_type:=1 \
    use_lcd:=true \
    lcd_no_optimize:=true
```

and in a separate terminal, run the same command for Hydra:

```
roslaunch hydra_dsg_builder uhumans2.launch \
     start_visualizer:=true \
     use_gt_frame:=false
```

:warning: To achieve the best results with Kimera-VIO, you should wait for the LCD vocabulary to finish loading before starting the rosbag.

#### Running Using VIO and DSG Loop Closures

First, start Kimera:

```
roslaunch kimera_vio_ros kimera_vio_ros kimera_vio_ros_uhumans2.launch online:=true viz_type:=1 \
     use_lcd:=true \
     lcd_no_detection:=true
```

and in a separate terminal, run the same command for Hydra:

```
roslaunch hydra_dsg_builder_ros uhumans2.launch \
     start_visualizer:=true \
     use_gt_frame:=false \
     enable_dsg_lcd:=true
```

:warning: To achieve the best results with Kimera-VIO, you should wait for the LCD vocabulary to finish loading before starting the rosbag.

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
