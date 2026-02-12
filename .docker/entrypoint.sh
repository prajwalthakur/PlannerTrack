#!/usr/bin/env bash
# Source ROS and the ros workspace
echo -e "Sourced ROS jazzy"
source /opt/ros/jazzy/setup.bash

# echo -e "Sourcing LD_LIBRARY_PATH"
# source /hpipm/examples/python/env.sh

if [ -f /workspace/ros_ws/install/setup.bash ]
then
  echo "Sourced ROS  workspace"
  source /workspace/ros_ws/install/setup.bash
  cd /workspace/ros_ws
fi

# Execute the command passed into this entrypoint
exec "$@"
