#!/bin/bash
source "$(dirname "$0")/base.sh"

stop_docker

mode="cpu"
while getopts 'ch' opt; do
    case "$opt" in
        c) mode="cpu" ;;
        ?|h) echo "Usage: $(basename "$0") [-c]"; exit 1 ;;
    esac
done
shift "$(($OPTIND -1))"

: "${PROJECT_ROOT:=$(pwd)}"

if [ "$mode" = "gpu" ]; then
  run_docker --gpus all \
    --mount type=bind,src="${PROJECT_ROOT}/workspace/ros_ws/src",target="/workspace/ros_ws/src" \
    --mount type=bind,src="/tmp/.X11-unix",target="/tmp/.X11-unix",readonly \
    -- bash
else
  run_docker \
    --mount type=bind,src="${PROJECT_ROOT}/workspace/ros_ws/src",target="/workspace/ros_ws/src" \
    --mount type=bind,src="/tmp/.X11-unix",target="/tmp/.X11-unix",readonly \
    -- bash
fi