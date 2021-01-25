
# Python Bindings of Kimera Scene Graph 
## About
This package creates Python bindings for the scene graph classes in Kimera-Semantics. Scene graphs files serialized from boost in C++ can be loaded in Python using this package. 

## Prerequisites
1. install pybind11 for catkin ```sudo apt-get install ros-melodic-pybind11-catkin```
2. compile kimera ```catkin build kimera```
3. compile scene graph classes ```catkin build kimera_scene_graph```
4. compile binding package ```catkin build kimera_scene_graph_python```
4. refresh workspace ```source ~/catkin_ws/devel/setup.bash```

## Example Usage of Loading Serialized Scene Graph
1. Generate a scene graph 
	- change the following paths in `kimera_scene_graph/launch/kimera_scene_graph_eval.launch` to where the files are located on your system
		- `tsdf_map_path`, `esdf_map_path`, `sparse_graph_path`. (note: serialized scene graph will be outputed to path specified in `scene_graph_output_path` of the launch file)

2. run `Kimera-Semantics/kimera_scene_graph_python/kimera_scene_graph_python/kimera_scene_graph_python_example.py` (see code for how to load the graph and access attributes)
  
# Preliminary instructions for loading scene graph file in python:

cd to `kimera_scene_graph_python/kimera_scene_graph_python`  

Example of how to load a scene graph in memory:

```import kimera_scene_graph_pybind as sg

graph = sg.SceneGraph()

sg.load_scene_graph(os.path.dirname(os.getcwd())+"/kimera_scene_graph_python/vxblx_files/goseek_scene_graph_scene1.vxblx", graph)```

At this point you should be able to call methods that have bindings defined in kimera_scene_graph_python_bindings.cpp on this graph instance. e.g. graph.getInterLayerEdgesMap()
