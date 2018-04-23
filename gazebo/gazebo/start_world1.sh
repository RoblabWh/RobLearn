#!/bin/bash

. gazebo_env.sh

gzserver world1.sdf
echo $GAZEBO_MODEL_PATH