#!/bin/bash

# Get base.sh funcs
source "$(dirname "$0")/base.sh"

stop_docker

run_docker mp_ros2:latest "/root/app.sh"