# ME461_LABS
## Some Instruction to launch the Gazebo Simulation:  
### Step 0: Install Gazebo     
	
 	sudo apt install ros-humble-gazebo-*  
  
### Step 1: Install TurtleBot3 via Debian Packages:       
	  
 	sudo apt install ros-humble-dynamixel-sdk      
	sudo apt install ros-humble-turtlebot3-msgs    
	sudo apt install ros-humble-turtlebot3    
  
### Step 2: Correctly setup the environment variable for the Turtlebot, add the following line to your bashrc:     
			
   	export TURTLEBOT3_MODEL=burger      
  
### Step 3: colcon build from root of workspace    
    
### Step 4: Use the following command to launch the simulation:        
		
  	ros2 launch sokoban gazebo_launch.py      
	
 This will launch the simulation, with a turtlebot3 at the centre of the world, along with a single box.  	  
  
### Step 5: You can now look at the topics being published:   
	ros2 topic list    
  
### Step 6: You can subscribe to these topics and use them in your applications    
  
### Step 7: If you wish to spawn more boxes in the environment at random locations, you can run the following command:    
  			  
	ros2 run spawn_service spawn_service --params-file src/spawn_service/params/spawn.yaml    
	
 This will read the spawn.yaml file placed in src/spawn_service/params/spawn.yaml. Infact you can go to this file and change the number of boxes you want to spawn.    
    
### Step 8: If you wish to spawn targets at random locations in the environment, you can run the following command:   
			  
   	ros2 launch spawn_service spawn_target.launch.py    
	
 This will spawn targets at random locations within the environment. Moreover, you can go to the spawn_target.yaml file to change the target object type and the number of targets you wish to spawn. The yaml file is placed at the same location where spawn.yaml was placed. Here is the location: src/spawn_service/params/spawn.yaml
	
### Note that currently following target types are available:    
	Donut  
	Infinity   
	Kare  
	b3gen   
		
	  

