#!/bin/bash
sudo nvidia-docker run \
  -it \
  --privileged=true \
  --net=host \
  --env="DISPLAY" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --rm \
  picky_robot
