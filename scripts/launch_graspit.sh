#!/bin/bash


if env | grep -q ^GRASPIT=
then
    echo "Using GRASPIT=" $GRASPIT
else
    export GRASPIT="/home/${USER}/.graspit"
	echo "Using GRASPIT=" $GRASPIT
fi

export GRASPIT_PLUGIN_DIR=$(dirname $(catkin_find libreachability_energy_plugin.so))


export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
graspit_simulator -p libgraspit_interface,libreachability_energy_plugin,libgrid_sample_plugin --node_name graspit