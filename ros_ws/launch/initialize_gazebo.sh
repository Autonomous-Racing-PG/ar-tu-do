# Set environment variable that tells gazebo where to look for models
_ROS_WS_DIR=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && cd .. && pwd)
export GAZEBO_MODEL_PATH=$_ROS_WS_DIR/src/simulation/racer_world/meshes/
unset _ROS_WS_DIR