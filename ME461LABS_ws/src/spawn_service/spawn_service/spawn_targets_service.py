'''
This script is used to spawn targets in the gazebo environment. The targets are spawned randomly in the environment.
The script reads the sdf file of the target and spawns the targets in random positions in the world.
'''
import rclpy
import os
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from ament_index_python.packages import get_package_share_directory
import random


class SpawnTargetsService(Node):
    def __init__(self):
        super().__init__('spawn_targets_service')
        self.declare_parameter('object_name', 'Donut')
        self.declare_parameter('number_of_target', 1)
        self.model_name = self.get_parameter('object_name').value
        self.number_of_target = self.get_parameter('number_of_target').value
        self.target_client = self.create_client(SpawnEntity, 'spawn_entity')
        while not self.target_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.get_logger().info('service is available')
        self.spawn_targets()

    
    def spawn_targets(self):
        pkg_path = os.path.join(get_package_share_directory('sokoban'))
        model_dir = os.path.join(pkg_path,'models')
        name = self.model_name
        number_of_target = self.number_of_target
        self.get_logger().info(f'Spawning {number_of_target} {name} targets')
        if name == 'Donut':
            model_file = os.path.join(model_dir, 'Donut', 'model.sdf')
        elif name == 'Infinity':
            model_file = os.path.join(model_dir, 'Infinity', 'model.sdf')
        elif name == 'Kare':
            model_file = os.path.join(model_dir, 'Kare', 'model.sdf')
        elif name == 'b3gen':
            model_file = os.path.join(model_dir, 'b3gen', 'model.sdf')
        else:
            self.get_logger().info('Invalid model name')
            # default model
            model_file = os.path.join(model_dir, 'Donut', 'model.sdf')
        
        futures = []
        for i in range(number_of_target):
            req = SpawnEntity.Request()
            with open(model_file, 'r') as file:
                req.xml = file.read()
            req.robot_namespace = name
            req.name = name + str(i)
            req.initial_pose.position.x = random.uniform(-2, 2)
            req.initial_pose.position.y = random.uniform(-2, 2)
            req.initial_pose.position.z = 0.0
            req.initial_pose.orientation.x = 0.0
            req.initial_pose.orientation.y = 0.0
            req.initial_pose.orientation.z = 0.0
            req.initial_pose.orientation.w = 1.0
            req.reference_frame = 'world'
            future = self.target_client.call_async(req)
            futures.append((future, req.name))

        for future, req_name in futures:
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                self.get_logger().info(f'Successfully spawned {req_name}')
            else:
                self.get_logger().info(f'Failed to spawn {req_name}')

def main(args=None):
    rclpy.init(args=args)
    spawn_targets_service = SpawnTargetsService()
    rclpy.spin(spawn_targets_service)
    spawn_targets_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        
