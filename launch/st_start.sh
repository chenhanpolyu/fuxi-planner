#!/bin/bash

{
	gnome-terminal -x bash -c "roslaunch fuxi-planner cc_pp_start.launch"
}&
sleep 10s
{
	gnome-terminal -x bash -c "roslaunch fuxi-planner map_ccst.launch"
}&
sleep 5s
{
	gnome-terminal -x bash -c "roslaunch fuxi-planner program_flyonce.launch"
}


