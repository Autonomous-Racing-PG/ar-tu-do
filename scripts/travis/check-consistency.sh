pathofscript=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
pathofvescsim=$pathofscript/../../ros_ws/src/simulation/vesc_sim

source $pathofscript/../../ros_ws/devel/setup.bash

mv $pathofvescsim/config/car_config.yaml $pathofvescsim/config/car_config_copy.yaml || { echo 'Could not find "car_config.yaml" in package "vesc_sim".' ; exit 1; }
rosrun vesc_sim params_to_yaml
cmp -s $pathofvescsim/config/car_config.yaml $pathofvescsim/config/car_config_copy.yaml || { echo 'Inconsisteny between "vesc_sim/include/car_config.h" and "vesc_sim/config/car_config.yaml" detected. Please run "rosrun vesc_sim params_to_yaml" to fix this.' ; exit 1; }
rm $pathofvescsim/config/car_config_copy.yaml