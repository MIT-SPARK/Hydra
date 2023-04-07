#!/bin/bash

if [[ $# -eq 0 ]] ; then
    echo 'Log path required!!'
    exit 1
fi

if [[ -d $1 ]] ; then
    echo 'Log path '$1' exists!'
    exit 1
fi


# Make Hydra Folders
mkdir -p $1
mkdir -p $1/frontend
mkdir -p $1/backend
mkdir -p $1/lcd
mkdir -p $1/pgmo
mkdir -p $1/topology

# Make Log Directory
mkdir -p $1/logs

roslaunch hydra_dsg_builder_ros hydra_robot.launch dsg_path:="$1" glog_to_file:=true glog_dir:="$1/logs" verbosity:=2 record_bag:=true exit_mode:=SERVICE enable_dsg_lcd:=true use_zmq_interface:=true 2> >(grep -v TF_REPEATED_DATA buffer_core)
