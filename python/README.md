### Installation

0. Build Hydra normally via colcon and make sure your colcon workspace is sourced

> **Note**<br>
> You can also manually build and install every listed dependency of Hydra (and Hydra) instead of building through colcon, but we do not maintain instructions on how to do this. Proceed at your own risk!

1. Make a virtual environment (you can use `venv` or whatever you want):

```bash
mkdir /path/to/environment
cd /path/to/environment
python3 -m virtualenv -p /usr/bin/python3 --download hydra # or some other environment name
```

2. Install the python package (n.b., this installs a non-editable version):

```bash
source /path/to/hydra/environment/bin/activate

# note that you may want to install a different version of spark_dsg than is installed automatically by the bindings
pip install /path/to/colcon_ws/src/spark_dsg
pip install /path/to/colcon_ws/src/hydra
```

### Running on MP3D Image Dataset

Point Hydra at the scene(s) you want to run:
```
hydra run mp3d /data/datasets/1LXtFkjw3qL [other scenes...]
```

This commmand runs against a dataset structured like this
```
.
├── camera_info.yaml
├── color
│   ├── rgb_0000000.png
│   ├── rgb_0000001.png
│   ├── rgb_0000002.png
│   ├── rgb_0000003.png
│   ├── rgb_0000004.png
│   ├── rgb_0000005.png
│   ├── rgb_0000006.png
│   ├── rgb_0000007.png
│   ├── rgb_0000008.png
│   └── rgb_0000009.png
├── depth
│   ├── depth_0000000.tiff
│   ├── depth_0000001.tiff
│   ├── depth_0000002.tiff
│   ├── depth_0000003.tiff
│   ├── depth_0000004.tiff
│   ├── depth_0000005.tiff
│   ├── depth_0000006.tiff
│   ├── depth_0000007.tiff
│   ├── depth_0000008.tiff
│   └── depth_0000009.tiff
├── labels
│   ├── labels_0000000.png
│   ├── labels_0000001.png
│   ├── labels_0000002.png
│   ├── labels_0000003.png
│   ├── labels_0000004.png
│   ├── labels_0000005.png
│   ├── labels_0000006.png
│   ├── labels_0000007.png
│   ├── labels_0000008.png
│   └── labels_0000009.png
└── poses.csv
```
with `camera_info.yaml` as
```yaml
cx: 320.0
cy: 240.0
fx: 320.00000000000006
fy: 320.00000000000006
height: 480
width: 640
```
and `poses.csv` as
```
timestamp_ns,tx,ty,tz,qw,qx,qy,qz
0,3.663007259368897,-3.0140908559163413,0.680124095082283,-0.6739747339486684,0.6739747339486684,0.2139113320953844,-0.2139113320953844
200000000,3.5630072355270386,-2.8724242051442466,0.680124095082283,-0.6739747339486684,0.6739747339486684,0.2139113320953844,-0.2139113320953844
400000000,3.4630072116851807,-2.730757554372152,0.680124095082283,-0.6739747339486684,0.6739747339486684,0.2139113320953844,-0.2139113320953844
600000000,3.3630071878433228,-2.589090903600057,0.680124095082283,-0.6739747339486684,0.6739747339486684,0.2139113320953844,-0.2139113320953844
800000000,3.263007164001465,-2.4474242528279624,0.680124095082283,-0.6739747339486684,0.6739747339486684,0.2139113320953844,-0.2139113320953844
1000000000,3.263007164001465,-2.4474242528279624,0.680124095082283,-0.6314914426627815,0.6314914426627815,0.3181486411155627,-0.3181486411155627
1200000000,3.263007164001465,-2.4474242528279624,0.680124095082283,-0.5730058467300353,0.5730058467300353,0.4143239066397151,-0.4143239066397151
1400000000,3.263007164001465,-2.4474242528279624,0.680124095082283,-0.5000000000000001,0.5000000000000001,0.5,-0.5
1600000000,3.0741182963053384,-2.4474242528279624,0.680124095082283,-0.5000000000000001,0.5000000000000001,0.5,-0.5
1800000000,2.8852294286092124,-2.4474242528279624,0.680124095082283,-0.5000000000000001,0.5000000000000001,0.5,-0.5
```

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
