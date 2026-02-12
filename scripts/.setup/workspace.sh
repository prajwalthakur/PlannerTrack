#!/bin/bash
# cd $WORKSPACE_PATH
# source /opt/ros/$ROS_DISTRO/setup.bash
# colcon build --symlink-install
# echo "source $WORKSPACE_PATH/install/setup.bash" >> /root/.bashrc
echo "export CUDA_HOME=/usr/local/cuda-12.6">> /root/.bashrc
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda-12.6/lib64">> /root/.bashrc
echo "export PATH=$PATH:/usr/local/cuda-12.6/bin">>/root/.bashrc
#echo "source /root/scripts/setup/env.sh" >> /root/.bashrc