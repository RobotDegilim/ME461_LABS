#!/usr/bin/env bash

docker stop me461_labs > /dev/null 2>&1 || true
docker rm me461_labs > /dev/null 2>&1 || true

docker run -d \
    --privileged \
    --net=host \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ~/robot_degilim_labs:/home/robot_degilim_labs/mnt/ \
    -v /dev:/dev \
    -e DISPLAY=${DISPLAY} \
    -u 1000 \
    -w /home/robot_degilim_labs/mnt/labs_ws \
    --name me461_labs\
    --restart unless-stopped \
    me461_labs:latest \
    sleep infinity

#extra convenient stuff
docker exec me461_labs bash -c "source /opt/ros/humble/setup.bash" 
docker exec me461_labs bash -c "colcon build --symlink-install"
docker exec me461_labs bash -c "echo 'source /opt/ros/humble/setup.bash' >> /home/robot_degilim_labs/.bashrc"
docker exec me461_labs bash -c "echo 'source /home/robot_degilim_labs/mnt/labs_ws/install/setup.bash' >> /home/robot_degilim_labs/.bashrc"
docker exec me461_labs bash -c "echo 'export PATH=\$PATH:/home/robot_degilim_labs/mnt/util/' >> /home/robot_degilim_labs/.bashrc"
docker exec me461_labs bash -c "echo 'export ROS_DOMAIN_ID=21' >> /home/robot_degilim_labs/.bashrc"