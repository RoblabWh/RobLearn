#!/bin/bash

. gazebo_env.sh

gzserver world2.sdf
echo $GAZEBO_MODEL_PATH