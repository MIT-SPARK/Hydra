# Object DB

This ROS package estimates the best transformation of a CAD model given a point
cloud with a corresponding CAD model.

## Dependencies
Other than the usual ROS dependencies, this package depends on [TEASER++](https://github.com/MIT-SPARK/TEASER-plusplus). 
You can use the provided `.rosinstall` file in the `install` folder:
``` sh
wstool init # Use unless wstool is already initialized
wstool merge object_db/install/object_db_ssh.rosinstall
wstool update
```
Or you can simply clone the TEASER++ repo to your catkin workspace.

## How it works
Under the hood, `object_db` is a ROS action server that will execute a callback function that tries to register a known object CAD model to the provided point cloud (if such CAD model exists). Refer to the source code of `executeCB()` function in `object_registration_server.cpp` file for complete details. A rough sketch of the code is the following:
- First, it will determine if the point cloud's semantic label is in the loaded object database.
- If not, it will:
  - Calculate the centroid errors with the point cloud directly.
- If yes, it will:
  - Calculate Harris 3D keypoints on the CAD model and the point cloud.
  - Match the keypoints with nearest-neighbor matching.
  - Use TEASER++ to register the CAD model with the point cloud.
  - Calculate the centroid errors with the registered CAD model.

A sample output log of `object_db` is shown below:

``` text
[ INFO] [1591561620.767236807]: Executing object registration server callback for 7.
[ INFO] [1591561620.767265353]: Dst size: 63.
[ INFO] [1591561620.767273252]: Src size: 10000.
[ INFO] [1591561620.767283923]: Calculate Harris keypoints
[ INFO] [1591561620.940628425]: Number of Harris keypoints: src -- 12, dst -- 7
[ INFO] [1591561620.940661679]: Generate correspondences.
[ INFO] [1591561620.940668488]: Solve with TEASER++.
*** [pmc heuristic: thread 1]   current max clique = 2,  time = 6.91414e-06 sec
[pmc heuristic]	 mc = 2
Created adjacency matrix in 1.5974e-05 seconds
[pmc: initial k-core pruning]  before pruning: |V| = 84, |E| = 49
[pmc: initial k-core pruning]  after pruning:  |V| = 84, |E| = 49
[pmc]  initial pruning took 6.19888e-06 sec
[pmc: sorting neighbors]  |E| = 98, |E_sorted| = 98
|V| = 14
-----------------------------------------------------------------------
[ INFO] [1591561620.941040831]: Solve complete!
[ INFO] [1591561620.941052523]: Solution valid!
R:
1 0 0
0 1 0
0 0 1
==========
t:
 23.2179
 23.8052
0.878101
==========
[ INFO] [1591561620.941940560]: Calculating GT errors.
[ INFO] [1591561620.941984350]: EST Centroid x: 23.209955, y: 24.233261, z: 0.832441
[ INFO] [1591561620.942031478]: object_db: Successful object alignment.
[ INFO] [1591561620.942043844]: Aligned object size: 497990
[ INFO] [1591561620.942051499]: Current KNOWN mean centroid error:   2.292107
[ INFO] [1591561620.942065277]: Current KNOWN centroid SD:           2.104384
[ INFO] [1591561620.942076326]: Current UNKNOWN mean centroid error: 0.310917
[ INFO] [1591561620.942087631]: Current UNKNOWN centroid SD:         0.240312
```

## How to run 
First, you should launch `kimera_scene_graph`. Then, you should launch `object_db`. Something like this should work: 
``` sh
roslaunch kimera_scene_graph kimera_scene_graph_eval.launch
```
And in another terminal, run
``` sh
roslaunch object_db object_db_eval.launch
```
Now, call the reconstruction service of `kimera_scene_graph`:

``` sh
rosservice call /kimera_scene_graph_eval_node/reconstruct_scene_graph "data: false"
```
You should be able to see outputs on the terminal you just launched `object_db` with.

In addition, the registered CAD models are published to ROS topics for visualization. Topics are with the names `\reg_pcl_${X}`, where X corresponds to the CAD model's ID.

## Parameters
Refer to the launch file for detailed explanation.

## Label-ID Mapping File Format
One of the required parameters of `object_db_` is a label-id mapping file. Its format is the following:
``` text
name,red,green,blue,alpha,id
Airventilation_3x,255,20,127,255,1
Airventilation_CrossSection,255,20,127,255,1
AirventilationGrid_A,255,20,127,255,1
AirventilationGrid_B,255,20,127,255,1
```
where `name` is the semantic name of the object, `red`, `green`, `blue`, `alpha` are the color values of the segmentation color of such object, and `id` is a numericall identifier for this category of object. It is possible for different objects within the same category to share the same ID. For example, as shown above, different air ventilation grids might share the same numerical ID. 
Set this file to be the same with the `semantic_label_2_color_csv_filepath` file parameter for `kimera_scene_graph` server.

## Ground Truth CSV File Format
The ground truth csv file contains the ground truth information of the objects.
``` text
name,pos_x,pos_y,pos_z,centroid_x,centroid_y,centroid_z
Airventilation_3x,6.5,3.5,25,6.5,3.5,25
Airventilation_3x (1),11.5,4,26.5,9.992,3.813,26.5
Airventilation_3x (10),23,4,50.5,23,3.813,48.872
```
The `pos_x`, `pos_y` and `pos_z` corresponds to the lower left corner of the bounding box of the object. `centroid_x`, `centroid_y`, `centroid_z` corresponds to the position of the centroid of the object.
You can get generate the GT data by accessing all the objects in the Unity simulator.
Note that currently we assume the coordinates in this file are in the unity frame.

## Database Format
The following represents the structure of a object database.
``` text
── sofa_db_1seat
   └── 7
       ├── model.obj
       └── model.ply
       └── model.mtl
```
Notice that currently only the `.obj` files are used.
Also, currently it only supports one model under one numerical ID.

## Future Work 
- Currently, `object_db` supports matching CAD with one category only. It is possible to extend it so that it supports matching CAD models with multiple kinds of point clouds.
- In addition, we assume each category only contains one CAD model.
- This package currently uses Harris 3D keypoint for keypoint detection, which is not the most cutting-edge 3D keypoint detector out there. There is immense potential here evaluating the performance of a learned keypoint detector or feature descriptor.
