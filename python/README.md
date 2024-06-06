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
pip install -r python/build_requirements.txt
pip install .
```

Optionally, to install an editable version:

```bash
source /path/to/hydra/environment/bin/activate
pip install "/path/to/catkin_ws/src/spark_dsg[viz]"  # can also be editable if desired
pip install -r python/build_requirements.txt
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
hydra habitat run /path/to/habitat/mp3d/17DRP5sb8fy/17DRP5sb8fy.glb
```

You can enable the open3d visualizer via:

```bash
hydra habitat run /path/to/habitat/mp3d/17DRP5sb8fy/17DRP5sb8fy.glb -v
```

and then running (in a different terminal):
```bash
hydra visualize
```

For some reason it appears habitat and the open3d visualizer are incompatible (if the open3d visualizer is launched as a child process of the process running habitat).
