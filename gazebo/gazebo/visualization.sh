#!/bin/bash

. gazebo_env.sh

gzclient world1.sdf
echo $GAZEBO_MODEL_PATH