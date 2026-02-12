#!/bin/bash
# base.sh (or wherever run_docker lives)
PROJECT_ROOT="${PROJECT_ROOT:-$PWD}"
BASE_NAME="${BASE_NAME:-mp_ros2}"
IMAGE_NAME="${IMAGE_NAME:-${BASE_NAME}}"
CONTAINER_NAME="${CONTAINER_NAME:-${BASE_NAME}}"

run_docker() {
    xhost +local:root

    # parse args: everything before '--' -> docker options, after '--' -> command inside container
    local -a docker_opts=()
    local -a cmd=()
    local saw_sep=0

    for arg in "$@"; do
        if [ "$arg" = "--" ]; then
            saw_sep=1
            continue
        fi
        if [ $saw_sep -eq 0 ]; then
            docker_opts+=("$arg")
        else
            cmd+=("$arg")
        fi
    done

    # default command if none provided
    if [ ${#cmd[@]} -eq 0 ]; then
        cmd=(bash)
    fi

    # remove any previous container with same name to avoid conflicts
    docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true

    docker run -it --privileged --network=host \
        --name "${CONTAINER_NAME}" \
        -e DISPLAY \
        -e QT_X11_NO_MITSHM=1 \
        -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}" \
        -e ROS_DISTRO="${ROS_DISTRO}" \
        -v "${PROJECT_ROOT}/scripts/deploy/app.sh:/root/app.sh" \
        "${docker_opts[@]}" \
        "${IMAGE_NAME}" \
        "${cmd[@]}"
}

stop_docker() {
    docker stop ${CONTAINER_NAME} && rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true
}