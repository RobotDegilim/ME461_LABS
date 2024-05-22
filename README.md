# ME461_LABS

This repository contains a self-contained environment for me461 students to learn the basics of ROS2 and Gazebo without having to worry about any installations or dependencies. It also provides the possibility to expand upon the environment in an easy distributable fashion.

The repo currently contains the following labs:

1. Sokoban Image Processing 
2. Sokoban Path Planning 

## Conventient First-Time Installation 

For maximum convenience:

1. Install the _Remote Development_ extension in VSCode.

2. Clone the repo to a directory named robot_degilim_labs on your desktop ```git clone https://github.com/RobotDegilim/ME461_LABS.git ~/Desktop/me461```

3. Open VSCode's command pallete (CTRL + SHIFT + P) and run ```Dev Containers: Rebuild and Reopen in Container```

4. Allow docker to launch GUIs by running ```xhost +local:docker```. This can optionally be _added to the .bashrc_ for extra convience. Ignore any warnings resulting from this command.

5. Wait for the building process to finish. The building process might take upwards of 10 mins depending on your internet connection.

6. Check your installation by running and ros2 or gazebo command. A good candidate is ```ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py``` since it requires all major components to be functioning properly (ROS2, Gazebo, and X11 server)

7. After building once, run ```Dev Containers: Reopen in Container``` in the command pallete.

## Enabling Nvidia Acceleration

(For PCs with NVIDIA GPUs) make sure you are using official NVIDIA drivers and install [NVIDIA Container Toolkit.](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) Test your installation with `sudo docker run --rm --runtime=nvidia --gpus all ubuntu nvidia-smi`

## Recommended Work Flow

For the best team work experience, create a Github Organization account and create your own fork of the repo. This way you will be able to collaborate in an online fashion and share your work easily with other team members.

## Building Using DOCKER CLI

ME461 students are recommended to use the convenience method outlined above. Curious students are encouraged to read the docker file, understand it, and then do everything through the terminal.

- To build the docker container (create an image) navigate to .Dockerfile directory and run ```docker build -t me461_labs -f ./labs.Dockerfile .``` 

- To run the docker container (instantiate and image) run the ```<path>/util/build_container_instance.sh``` script.

## Launching the Game

- To spawn a turtlebot and a camera in Gazebo run ```ros2 launch sokoban gazebo_launch.py world:="<world_name>.world"```. The camera stream is published on the topic `world_cam/image_raw`

- Note that the world argument defaults to empty.world. The following worlds are available as of currently:
    1. empty.world
    2. easymode_v1.world
    3. model1_v1.world
    4. model2_v1.world


## TO DO LIST 
1. Add ability to build in diff paths in the script
2. port second(spawninig scripts) package
3. 