#!/bin/bash
docker build --rm \
    --build-arg USER_UID="$(id -u)" \
    --build-arg USER_GID="$(id -g)" \
    $@ -t planner_track:latest -f "$(dirname "$0")/../../.docker/planner_track.Dockerfile" "$(dirname "$0")/../.."