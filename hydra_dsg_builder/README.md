# Kimera DSG-Builder

## Running online

Steps to running with ground truth for the uhumans2 office dataset:

  1. Grab the uhumans2 dataset (if you haven't already). You can probably just use the office scene without any humans (`uHumans2_office_s1_00h.bag`)

  2. Start up the DSG builder

    roslaunch hydra_dsg_builder uhumans2_incremental_dsg.launch start_visualizer:=true

  3. Start the bag:

    cd /path/to/uhumans2/office/scene
    rosbag play uHumans2_office_s1_00h.bag --clock --pause -r 0.5

You way want to try the following arguments for the `uhumans2_incremental_dsg` launch file:

  - `use_gt_frame:=false` switches the code to listen for the TF from Kimera-VIO
  - `min_glog_level:=0` will enable helpful development output.
  - `verbosity:=3` will enable a lot more development output
  - `debug:=true` will run the dsg builder in gdb (hopefully not necessary)

You can find parameters for the front-end [here](config/dsg_frontend_config.yaml)
and for the back-end [here](config/dsg_backend_config.yaml)

Important parameters:

  - `pgmo/*`: all pgmo settings for the `MeshFrontend` and `KimeraPgmoInterface` (frontend and backend configs respectively)
  - `dsg/add_places_to_deformation_graph`: controls place-based factors
  - `dsg/optimize_on_lc`: allows disabling optimization
