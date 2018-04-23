#!/bin/bash

. gazebo_env.sh

gzclient
echo $GAZEBO_MODEL_PATH
