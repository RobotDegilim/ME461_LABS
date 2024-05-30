import rclpy
import rclpy.node as rn
import rclpy.parameter as rp

from gazebo_msgs.srv import SpawnEntity, GetModelList
from ament_index_python.packages import get_package_share_directory

import xacro 
import random
import os


class SpawnObjects(rn.Node):
    
    def __init__(self):
        super().__init__('object_spawner')
        
        # Declare Parameters
        self.declare_parameter('is_random', False)
        self.declare_parameter('spawn_box', False)
        self.declare_parameter('num_box', 0)
        self.declare_parameter('spawn_target', False)
        self.declare_parameter('target_type', 'donut')
        self.declare_parameter('num_target', 0)   
        
        # Init Used Services
        self.spawn_client = self.create_client(SpawnEntity, 'spawn_entity')
        self.model_grabber = self.create_client(GetModelList, 'get_model_list')
        
        # Check if the service is available
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.get_logger().info('service is available')
        
        spawn_box_val = self.get_parameter('spawn_box').get_parameter_value().bool_value
        spawn_target_val = self.get_parameter('spawn_target').get_parameter_value().bool_value
        target_type_val = self.get_parameter('target_type').get_parameter_value().string_value
        
        self.objs = {}        
        
        future = self.model_grabber.call_async(GetModelList.Request())
        rclpy.spin_until_future_complete(self, future)
        spawned_models = str(future.result().model_names)
        
        curr_box_num = 0 
        curr_target_num = 0
        boxes = spawned_models.rfind('box')
        targets = spawned_models.rfind('target')
        
        if boxes != -1:
            curr_box_num = int(spawned_models[boxes+3]) + 1
        if targets != -1:
            curr_target_num = int(spawned_models[targets+6]) + 1
        
        if spawn_box_val:
            box_file = self.get_model_file('box')
            self.objs['box'] = (self.get_parameter('num_box').value, box_file.toxml(), curr_box_num)
        if spawn_target_val: 
            target_file = self.get_model_file(target_type_val)
            self.objs['target'] = [self.get_parameter('num_target').value , target_file, curr_target_num]
        
 
    def spawn_objects_(self, is_random, objects):
    
        futures = []
        object_names = objects.keys()
        
        if is_random:
            for obj in object_names:
                for i in range(objects[obj][0]):
                    req = SpawnEntity.Request()
                    req.xml = objects[obj][1]
                    req.robot_namespace = obj
                    req.name = obj + str(i+objects[obj][2])
                    req.initial_pose.position.x = random.uniform(-3, 3)
                    req.initial_pose.position.y = random.uniform(-3, 3)
                    req.initial_pose.position.z = 0.0
                    req.initial_pose.orientation.x = 0.0
                    req.initial_pose.orientation.y = 0.0
                    req.initial_pose.orientation.z = 0.0
                    req.initial_pose.orientation.w = 1.0
                    req.reference_frame = 'world'
                    future = self.spawn_client.call_async(req)
                    futures.append((future, req.name))
                
        for future, req_name in futures:
           rclpy.spin_until_future_complete(self, future)
           if future.result() is not None:
               self.get_logger().info(f'Successfully spawned {req_name}')
           else:
               self.get_logger().info(f'Failed to spawn {req_name}')

    def spawn_objects(self):
        is_random_val = self.get_parameter('is_random').get_parameter_value().bool_value
        self.spawn_objects_(is_random_val, self.objs)
        
    def get_model_file(self, model):

        model_file = ''
        
        pkg_path = os.path.join(get_package_share_directory('sokoban'))
        model_dir = os.path.join(pkg_path,'models')
        model_name = model.lower()
        
        if model_name == 'donut':
            model_file = os.path.join(model_dir, 'Donut', 'model.sdf')
        elif model_name == 'eight':
            model_file = os.path.join(model_dir, 'Infinity', 'model.sdf')
        elif model_name == 'square':
            model_file = os.path.join(model_dir, 'Kare', 'model.sdf')
        elif model_name == 'triangle':
            model_file = os.path.join(model_dir, 'b3gen', 'model.sdf')
        elif model_name == 'box':
            pkg_path = os.path.join(get_package_share_directory('sokoban'))
            xacro_file = os.path.join(pkg_path,'urdf','box.urdf.xacro')
            return xacro.process_file(xacro_file)
        else:
            print('Invalid model name')
            print('Resorting to Default Model')
            model_file = os.path.join(model_dir, 'Donut', 'model.sdf')
        
        with open(model_file, 'r') as file:
                return file.read()

   
def main(args=None):
    rclpy.init(args=args)
    spawn_service = SpawnObjects()
    spawn_service.spawn_objects()
    spawn_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
