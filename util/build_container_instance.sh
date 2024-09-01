#!/usr/bin/env bash

# Get the directory of the script
ME461_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && cd .. && pwd )
echo "Using path: ${ME461_DIR}"

docker stop me461_labs > /dev/null 2>&1 || true
docker rm me461_labs > /dev/null 2>&1 || true

docker run -d \
    --privileged \
    --net=host \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ${ME461_DIR}:/home/me461/mnt/ \
    -v /dev:/dev \
    -e DISPLAY=${DISPLAY} \
    -u 1000 \
    -w /home/me461/mnt/labs_ws \
    --name me461_labs\
    --restart unless-stopped \
    me461_labs:latest \
    sleep infinity

#extra convenient stuff
docker exec me461_labs /bin/bash ../util/init_env.sh