# ME461_LABS

This repository contains an self-environment for me461 to learn the basics opf ROS2 and Gazebo without having to worry about any installations or dependencies. tt also provides the possibility to expand upon the environment in an easy distributable fashion.

The repo currently contains the following labs:

1. Sokoban Image Processing 
2. Sokoban Path Planning 

## Conventient First-Time Installation 

For maximum convenience:

1. Install the _Remote Development_ extension in VSCode.

2. Clone the repo to a directory named robot_degilim_labs on your desktop ```git clone https://github.com/RobotDegilim/ME461_LABS.git ~/Desktop/robot_degilim_labs```

3. Open VSCode's command pallete (CTRL + SHIFT + P) and run ```Dev Containers: Rebuild and Reopen in Container```

4. Allow docker to launch GUIs by running ```xhost +local:docker```. This can optionally be added to the .bashrc for extra convience. Ignore any warnings resulting from this command.

5. Wait for the building process to finish. The building process might take upwards of 10 mins depending on your internet connection.

6. Check your installation by running and ros2 or gazebo command. A good candidate is ```ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py``` since it requires all major components to be functioning properly (Ros2, Gazebo, and X11 server)

7. After building once, run ```Dev Containers: Reopen in Container``` in the command pallete.

## Enabling Nvidia Acceleration

(For PCs with NVIDIA GPUs) make sure you are using official NVIDIA drivers and install [NVIDIA Container Toolkit.](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) Test your installation with `sudo docker run --rm --runtime=nvidia --gpus all ubuntu nvidia-smi`

## Recommended Work Flow

For the best team work experience, create a Github Organization account and create your own fork of the repo. This way you will be able to collaborate without having to meet physically. 

## Building Using DOCKER CLI

ME461 students are recommended to use the convenience method outlined above. Curious students are encouraged to read the docker file, understand it, and then do everything through the terminal

- To build the docker container navigate to .Dockerfile directory and run ```docker build -t me461_labs -f ./labs.Dockerfile .``` 

- To first create the container ``` docker run fill this part later```

### TO DO LIST 


