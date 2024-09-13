import rclpy
import rclpy.node as rn

from gazebo_msgs.srv import SpawnEntity, GetModelList
from ament_index_python.packages import get_package_share_directory
from .util_functions.util import get_model_file

import os
import json
import random

class SpawnObjects(rn.Node):
    
    def __init__(self):
        super().__init__('object_spawner')
         
        #* Declare Parameters
        self.declare_parameter('spawn_box', False)
        self.declare_parameter('num_box', 0)
        self.declare_parameter('spawn_target', False)
        self.declare_parameter('target_type', 'donut')
        self.declare_parameter('num_target', 0)   
        self.declare_parameter('x_rand', 3) #* Maximum possible coordinate in meters 
        self.declare_parameter('y_rand', 3) #* Used for random number generation
        self.declare_parameter('spawn_from_json', False)
                
        #* Init Used Services
        self.spawn_client = self.create_client(SpawnEntity, 'spawn_entity')
        self.model_grabber = self.create_client(GetModelList, 'get_model_list')
        
        #* Check If the Services are Available
        while not self.spawn_client.wait_for_service(timeout_sec=1.0) and not self.model_grabber.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('services not available, waiting again...')
        self.get_logger().info('services are available')
        
        #* Initiliaze Spawning Info for Each Object
        spawn_from_json_val = self.get_parameter('spawn_from_json').value
        objs = self.objects_random() if not spawn_from_json_val else self.objects_from_json()
        
        #* Spawn Objects
        self.spawn_objects(objs)
    
    def objects_random(self) -> dict:
        '''
            Fill a dictionary with random object locations.
            
            Returns:  
                objs: necessary information to spawn each object.
        ''' 
        
        objs = {}       

        #* Retrieve Necessary Parameter Values
        spawn_box_val = self.get_parameter('spawn_box').get_parameter_value().bool_value
        spawn_target_val = self.get_parameter('spawn_target').get_parameter_value().bool_value
        target_type_val = self.get_parameter('target_type').get_parameter_value().string_value
        num_box_val = self.get_parameter('num_box').value
        num_targets_val = self.get_parameter('num_target').value
        x_rand_val = self.get_parameter('x_rand').value
        y_rand_val = self.get_parameter('y_rand').value
        
        #* Retrive Number of already Spawned Boxes
        curr_box_num, curr_target_num = self.get_spawned_box_target_number()
        
        #* Fill the Dictionary
        if spawn_box_val:
            box_file = get_model_file('box')
            for i in range(num_box_val):
                objs['box'+str(i+curr_box_num)] = (box_file, random.uniform(-x_rand_val,x_rand_val), random.uniform(-y_rand_val,y_rand_val))
        
        if spawn_target_val: 
            target_file = get_model_file(target_type_val)
            for i in range(num_targets_val):
                objs['target'+str(i+curr_target_num)] = (target_file, random.uniform(-x_rand_val,x_rand_val), random.uniform(-y_rand_val,y_rand_val))
        
        return objs
    
    def objects_from_json(self) -> dict:
        '''
            Fill a dictionary with locations from the JSON file.
            
            Returns:  
                objs: necessary information to spawn each object.
        ''' 
        
        objs = {}       
        
        #* Retrive Number of already Spawned Boxes
        curr_box_num, curr_target_num = self.get_spawned_box_target_number()
        
        #* Read JSON File
        json_file_path = os.path.join(get_package_share_directory('spawn_objects'), 'launch', 'objects.json')
        with open(json_file_path, 'r') as file:
            targets_data = json.load(file)
        
        #* Fill the Dictionary
        target_counter = 0
        box_counter = 0
        for target_info in targets_data['targets']:
            
            model_name = target_info['name']
            model_file = get_model_file(model_name)
            
            if model_name == 'box':
                objs['box' + str(box_counter + curr_box_num)] = (model_file, target_info['x'], target_info['y'])
                box_counter += 1 
            else:
                objs['target'+ str(target_counter + curr_target_num)] = (model_file, target_info['x'], target_info['y'])
                target_counter += 1
        
        return objs
       
    def get_spawned_box_target_number(self) -> tuple:
        '''
            Retrieves the count of already spawned boxes.
            
            Returns:  
                tuple: spawned_boxes, spawned_targets.
        ''' 
        
        #* Obtain Complete List of Spawned Objects as a String
        future = self.model_grabber.call_async(GetModelList.Request())
        rclpy.spin_until_future_complete(self, future)
        spawned_models = str(future.result().model_names)
        
        curr_box_num = 0 
        curr_target_num = 0
        
        #* Find Index of Last Spawned Box or Target
        boxes = spawned_models.rfind('box')
        targets = spawned_models.rfind('target')
        
        #* Retrieve Number of Spawned Boxes and Targets
        if boxes != -1:
            curr_box_num = int(spawned_models[boxes+3:].split("',")[0].rstrip("']")) + 1
        if targets != -1:
            curr_target_num = int(spawned_models[targets+6:].split("',")[0].rstrip("']")) + 1
        
        return (curr_box_num, curr_target_num)
    
    def spawn_objects(self, objects: dict):
        '''
            Calls a Gazebo service that spawns the objects
            
            Args: 
                objects: a dictionary containing the name of the object, urdf 
                            definition as a string, and position in the x-y plane.
        '''
        
        futures = []
        object_names = objects.keys()
        
        #* Fill Request Data
        for obj in object_names:
                req = SpawnEntity.Request()
                req.xml = objects[obj][0]
                req.robot_namespace = obj
                req.name = obj
                req.initial_pose.position.x = objects[obj][1]
                req.initial_pose.position.y = objects[obj][2]
                req.initial_pose.position.z = 0.0
                req.initial_pose.orientation.x = 0.0
                req.initial_pose.orientation.y = 0.0
                req.initial_pose.orientation.z = 0.0
                req.initial_pose.orientation.w = 1.0
                req.reference_frame = 'world'
                future = self.spawn_client.call_async(req)
                futures.append((future, req.name))
        
        #* Send Requests     
        for future, req_name in futures:
           rclpy.spin_until_future_complete(self, future)
           if future.result() is not None:
               self.get_logger().info(f'Successfully spawned {req_name}')
           else:
               self.get_logger().info(f'Failed to spawn {req_name}')

    
def main(args=None):
    rclpy.init(args=args)
    #* Instantiate and then Immediatly Kill the Node after the Service Call is Done
    spawn_service = SpawnObjects()
    spawn_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()