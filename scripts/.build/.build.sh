#!/bin/bash
docker build --rm  $@ -t planner_track:latest -f "$(dirname "$0")/../../.docker/planner_track.Dockerfile" "$(dirname "$0")/../.."