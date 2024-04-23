# ME461_LABS
## Some Instruction to launch the Gazebo Simulation:  
### Step 0: Install Gazebo     
	
 	sudo apt install ros-humble-gazebo-*  
  
### Step 1: Install TurtleBot3 via Debian Packages:       
	  
 	sudo apt install ros-humble-dynamixel-sdk      
	sudo apt install ros-humble-turtlebot3-msgs    
	sudo apt install ros-humble-turtlebot3    
  
### Step 2: Correctly setup the environment variable for the Turtlebot, add the following line to your bashrc:     
			
   	export TURTLEBOT3_MODEL=waffle       
  
### Step 3: colcon build from root of workspace      
	
colcon build       
    	  
### Step 4: Use the following command to launch the simulation:        
		
  	ros2 launch sokoban gazebo_launch.py      
	
 This will launch the simulation, with a turtlebot3 at the centre of the world, along with a single box.  	  
  
### Step 5: You can now look at the topics being published:   
	ros2 topic list    
  
### Step 6: You can subscribe to these topics and use them in your applications    
  
### Step 7: If you wish to spawn more boxes in the environment at random locations, you can run the following command:    
  			  
	 ros2 launch spawn_service spawn_box.launch.py object_name:='box' number_of_boxes:=2  
	
 You can modify the number_of_boxes parameter during launch from terminal, however currently we only have 'box' object in this sokoban game, so donot change its value.        
     
### Step 8: If you wish to spawn targets at random locations in the environment, you can run the following command:   
			  
   	ros2 launch spawn_service spawn_target.launch.py object_name:='Donut' number_of_target:=5            
	
 This will spawn targets at random locations within the environment.You are free to modify the object_name to any of the available target types as mentioned below. Similarly, you are free to change number_of_targets parameter to your desired number of targets  
	
### Note that currently following target types are available:    
	Donut  
	Infinity   
	Kare  
	b3gen   
### Some Common Errors:  
Error #1:   
		[ERROR] [launch]: Caught exception in launch (see debug for traceback): Caught multiple exceptions when trying to load file of format [py]:  
		-KeyError: 'TURTLEBOT3_MODEL'  
		-InvalidFrontendLaunchFileError: The launch file may have a syntax error, or its format is unknown  
 		
 	To resolve this, you must set the turtlebot3 model:
 		
		export TURTLEBOT3_MODEL=waffle
  
Error #2: 
If you get the following error when trying to launch the simulation, 
		[gzclient-3] gzclient: /usr/include/boost/smart_ptr/shared_ptr.hpp:728: typename boost::detail::sp_member_access<T>::type boost::shared_ptr<T>::operator->() const [with T =   	   
		gazebo::rendering::Camera; typename boost::detail::sp_member_access<T>::type = gazebo::rendering::Camera*]: Assertion `px != 0' failed.  
	  
	To resolve this, 
		source /usr/share/gazebo/setup.sh


	  

