SCRIPTPATH="$( cd "$(dirname "$0")" ; pwd -P )"

export GAZEBO_MODEL_PATH=$SCRIPTPATH/models:$GAZEBO_MODEL_PATH
