#!/usr/bin/env bash

# Runs a docker container with the image created by build.bash
# Requires:
#   docker
#   an X server

# Set the image name
IMG_NAME=dvrk

CONTAINER_NAME="$(tr '/' '_' <<< "$IMG_NAME")_container"

# Start the container
docker run --rm -it --name $CONTAINER_NAME --network host \
    -e DISPLAY -e TERM -e QT_X11_NO_MITSHM=1 $IMG_NAME bash
