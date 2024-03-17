# ME461_LABS
## Some Instruction to launch the Gazebo Sim:
Step 1: Install Turtlebot3 package
Step 2: Correctly setup the environment variable for the Turtlebot, add the following line to your bashrc: export TURTLEBOT3_MODEL=burger
Step 3: colcon build from root of workspace
Step 4: Use the following command to launch: ros2 launch sokoban gazebo_launch.py
Step 5: You can now look at the topic being published: ros2 topic list
step 6: You can subscribe to these topics and use them in your applications
