### Installation

0. Build Hydra normally via catkin and make sure your catkin workspace is sourced

> **Note**<br>
> You can also manually build and install every listed dependency of Hydra (and Hydra) instead of building through catkin, but we do not maintain instructions on how to do this. Proceed at your own risk!

1. Make a virtual environment (you can use `venv` or whatever you want):

```bash
mkdir /path/to/environment
cd /path/to/environment
python3 -m virtualenv -p /usr/bin/python3 --download hydra # or some other environment name
```

2. Install the python package (n.b., this installs a non-editable version):

```bash
source /path/to/hydra/environment/bin/activate
cd "/path/to/catkin_ws/src/hydra"

# note that you may want to install a different version of spark_dsg than is installed automatically by the bindings
# pip install "/path/to/catkin_ws/src/spark_dsg[viz]"

pip install .  # use `-e` to enable an editable install
```

> **:warning: Warning**<br>
> Some versions of `setuptools` may not install listed build requirements when making an editable install. You may have to install the listed build requirements beforehand if this is the case (see [pyproject.toml](pyproject.toml]) for details).

### Running on MP3D Image Dataset

Point Hydra at the scene(s) you want to run:
```
hydra run /data/datasets/1LXtFkjw3qL [other scenes...]
```

You may find other arguments useful:
```
hydra run /data/datasets/1LXtFkjw3qL --openset-model "ViT-L/14" -m 500 -o ~/test_python_clip -v
```

In this case:
  - `--openset-model "ViT-L/14"` turns on assigning features (produced by clip) to nodes
  - `-v` turns on publishing the scene graph via zmq
  - `-m 500` limits the total number of images to 500
  - `-o ~/test_python_clip` sets the top-level output directory (each scene will be saved in a subdirectory)

### Setting up Habitat

Set up habitat via [conda](https://github.com/facebookresearch/habitat-sim#installation) first, and then pip install Hydra into that environment.

**For Nathan:** Incus requires configuring EGL. Make the file `usr/share/glvnd/egl_vendor.d/10_nvidia.json` containing (see [here](https://github.com/facebookresearch/habitat-sim/issues/1671)):
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
