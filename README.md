## Hydra

<div align="center">
    <img src="doc/media/hydra.GIF">
</div>

This repository contains code to incrementally build 3D Scene Graphs in real-time and is based on the papers:
  - ["Hydra: A Real-time Spatial Perception System for 3D Scene Graph Construction and Optimization"](http://www.roboticsproceedings.org/rss18/p050.pdf)
  - ["Foundations of Spatial Perception for Robotics: Hierarchical Representations and Real-time Systems"](https://journals.sagepub.com/doi/10.1177/02783649241229725)

If you find this code relevant for your work, please consider citing one or both of these papers. A bibtex entry is provided below:

```bibtex
@article{hughes2022hydra,
    title={Hydra: A Real-time Spatial Perception System for {3D} Scene Graph Construction and Optimization},
    fullauthor={Nathan Hughes, Yun Chang, and Luca Carlone},
    author={N. Hughes and Y. Chang and L. Carlone},
    booktitle={Robotics: Science and Systems (RSS)},
    pdf={http://www.roboticsproceedings.org/rss18/p050.pdf},
    year={2022},
}

@article{hughes2024foundations,
    title={Foundations of Spatial Perception for Robotics: Hierarchical Representations and Real-time Systems},
    fullauthor={Nathan Hughes and Yun Chang and Siyi Hu and Rajat Talak and Rumaisa Abdulhai and Jared Strader and Luca Carlone},
    author={N. Hughes and Y. Chang and S. Hu and R. Talak and R. Abdulhai and J. Strader and L. Carlone},
    journal={The International Journal of Robotics Research},
    doi={10.1177/02783649241229725},
    url={https://doi.org/10.1177/02783649241229725},
    year={2024},
}
```

## Table of Contents

- [News](#news)
- [Installation and Running](#installation-and-running)
    - [Python Bindings](#hydra-python-bindings)
    - [Evalaution](#hydra-evaluation)
- [Filing Issues](#filing-issues)
- [Acknowledgements](#acknowledgements)

## News

**Update (07/09/25):** We've archived the ROS1 version of Hydra and switched to the ROS2 version by default. See [this branch](https://github.com/MIT-SPARK/Hydra/tree/archive/ros_noetic) for the pinned version of the code if you need the ROS1 version for any reason. We are unlikely to support any issues that come up with the archived ROS1 version.

<details>
<summary>Previous News</summary>

**Update (01/28/25):** We've released a new version of Hydra. This involves the following changes:
  - Open-set semantic capabilities used by downstream projects (i.e., [Khronos](https://github.com/MIT-SPARK/Khronos) and [Clio](https://github.com/MIT-SPARK/Clio))
  - A clear separation between colors and semantic labels. Hydra no longer assigns colors to nodes when building a scene graph; this is done by the visualizer
  - Instructions for using a real semantic segmentation model via [semantic_inference](https://github.com/MIT-SPARK/semantic_inference.git)
  - Internal refactoring and cleanup

> **Node**<br>
> We've updated our rosinstall file to point to the `main` branch of Kimera-PGMO. Please make sure you have the right branch checked out!

**Update (06/26/24):** We've released the latest version of Hydra.
This release also includes the following features:
  - Room category classification networks from our most recent paper, available [here](https://github.com/MIT-SPARK/Hydra-GNN)
  - Python bindings around Hydra and our interface for running with the habitat simulator (details below)
  - Updates to Hydra to use [config_utilities](https://github.com/MIT-SPARK/config_utilities)

> **Note**<br>
> We've changed (and simplified) the dependencies required to build Hydra. This includes moving towards the ROS packaged version of GTSAM, dropping Kimera-Semantics and replacing voxblox with [spatial_hash](https://github.com/MIT-SPARK/Spatial-Hash).
> Please make sure to double-check the rosinstall files to make sure you have all new dependencies (and feel free to remove unused old dependencies).

**Update (06/26/23):** We've released initial changes from the our newest paper.
We also plan to release additional code, most notably for training the room classification networks and GNN-based descriptors as described in the above paper.
We will link to the new repository once this is done.

> **Note**<br>
> As part of the this release, we have moved ROS related code to a new repository located [here](https://github.com/MIT-SPARK/Hydra-ROS). This code (and our installation process) still do rely on the ROS ecosystem.

</details>

## Installation and Running

Hydra has been tested on Ubuntu 24.04 and ROS2 Jazzy.
Please refer to our guide [here](https://github.com/MIT-SPARK/Hydra-ROS/tree/main?tab=readme-ov-file#getting-started) for installation instructions.

### Hydra Python Bindings

See [here](python/README.md) for information

### Hydra Evaluation

See [here](eval/README.md) for information

## Filing Issues

Please understand that this is research code maintained by busy graduate students, **which comes with some caveats**:
  1. We do our best to maintain and keep the code up-to-date, but things may break or change occasionally
  2. We do not have bandwidth to help adapt the code to new applications
  3. The documentation, code-base and installation instructions are geared towards practitioners familiar with ROS and 3D scene graph research.

> **:warning: Warning**<br>
> We don't support other platforms. Issues requesting support on other platforms (e.g., Ubuntu 18.04, Windows) will be summarily closed.

Depending on the nature of the issue, it may be helpful to browse [this](doc/debugging.md) page about debugging Hydra first.

Thank you in advance for your understanding!

## Acknowledgements

This work was partially funded by the AIA CRA FA8750-19-2-1000, ARL DCIST CRA W911NF-17-2-0181, and ONR RAIDER N00014-18-1-2828.

#### Disclaimer

Research was sponsored by the United States Air Force Research Laboratory and the United States Air Force Artificial Intelligence Accelerator and was accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views and conclusions contained in this document are those of the authors and should not be interpreted as representing the official policies, either expressed or implied, of the United States Air Force or the U.S. Government. The U.S. Government is authorized to reproduce and distribute reprints for Government purposes notwithstanding any copyright notation herein.
