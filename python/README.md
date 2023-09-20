## Hydra python

pybind11 bindings around Hydra.

### Acknowledgements and Disclaimer

**Acknowledgements:** This work was partially funded by the AIA CRA FA8750-19-2-1000, ARL DCIST CRA W911NF-17-2-0181, and ONR RAIDER N00014-18-1-2828.

**Disclaimer:** Research was sponsored by the United States Air Force Research Laboratory and the United States Air Force Artificial Intelligence Accelerator and was accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views and conclusions contained in this document are those of the authors and should not be interpreted as representing the official policies, either expressed or implied, of the United States Air Force or the U.S. Government. The U.S. Government is authorized to reproduce and distribute reprints for Government purposes notwithstanding any copyright notation herein.

### Installation

0. Make sure Hydra builds normally via catkin

1. Make a virtual environment (you can use `venv` or whatever you want):

```bash
mkdir /path/to/environment
cd /path/to/environment
python3 -m virtualenv -p /usr/bin/python3 --download hydra # or some other environment name
```

2. Install the python package (n.b., this installs a non-editable version):

```bash
source /path/to/hydra/environment/bin/activate

# required to expose DSG python bindings
pip install "/path/to/catkin_ws/src/spark_dsg[viz]"  # can also be editable if desired

git clone git@github.mit.edu:SPARK/hydra_python.git
cd hydra_python
pip install -r build_requirements.txt
pip install .
```

Optionally, to install an editable version:

```bash
source /path/to/hydra/environment/bin/activate
pip install "/path/to/catkin_ws/src/spark_dsg[viz]"  # can also be editable if desired
pip install -r build_requirements.txt
pip install git+https://github.com/ros/catkin.git@noetic-devel
pip install -e .
```

### Setting up Habitat

Set up habitat via [conda](https://github.com/facebookresearch/habitat-sim#installation)

**For Nathan:** LXC requires configuring EGL. Make the file `usr/share/glvnd/egl_vendor.d/10_nvidia.json` containing (see [here](https://github.com/facebookresearch/habitat-sim/issues/1671)):
```
{
    "file_format_version" : "1.0.0",
    "ICD" : {
        "library_path" : "libEGL_nvidia.so.0"
    }
}
```

### Running against Habitat

As long as you have mp3d set up on your machine and, then source your virtual environment and:

```bash
hydra run /path/to/habitat/mp3d/17DRP5sb8fy/17DRP5sb8fy.glb
```

You can enable the open3d visualizer via:

```bash
hydra run /path/to/habitat/mp3d/17DRP5sb8fy/17DRP5sb8fy.glb -v
```

and then running (in a different terminal):
```bash
hydra visualize
```

For some reason it appears habitat and the open3d visualizer are incompatible (if the open3d visualizer is launched as a child process of the process running habitat).
